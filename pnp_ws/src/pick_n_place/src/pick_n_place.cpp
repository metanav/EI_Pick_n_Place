#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include <object_recognition_msgs/msg/object_type.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>

#include <std_srvs/srv/trigger.hpp>

#define NUM_BINS 2
#define BIN_DIST 0.38
#define BIN_0_ANGLE (7.0f * M_PI/18) 
#define BIN_1_ANGLE -(7.0f * M_PI/18) // -70 deg the from world frame's x-axis
#define BIN_H 0.10
#define BIN_W 0.075
#define BIN_L 0.075
				      
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_n_place");

class TriggerMotionPlanningService : public rclcpp::Node
{
public:
  bool motion_planning_enabled = false;

  TriggerMotionPlanningService() : Node("trigger_motion_planning_server")
  {
      service_ = create_service<std_srvs::srv::Trigger>( "trigger_motion_planning",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
      {
          (void) request;
	  this->motion_planning_enabled = !this->motion_planning_enabled;
          RCLCPP_INFO(LOGGER, "TriggerMotionPlanningService: %s.", 
			  this->motion_planning_enabled ? "on" : "off");  
	  response->success = true;
      });
  }
private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};


class SpatialDetectionSubscriber : public rclcpp::Node
{
public:
  struct DetectionPose {
    std::string class_id;
    geometry_msgs::msg::Pose pose;
  };

  std::vector<DetectionPose> getLatestDetections();

  SpatialDetectionSubscriber() : Node("spatial_detection_subscriber")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    try {
      subscription_.subscribe(this, "/ei_yolov5/spatial_detections");
      cache_.setCacheSize(100);
      cache_.connectInput(subscription_);
    } catch (const std::exception &ex) {
      RCLCPP_INFO( LOGGER, "Error: %s", ex.what());
    }
  }

private:
  message_filters::Subscriber<depthai_ros_msgs::msg::SpatialDetectionArray> subscription_;
  message_filters::Cache<depthai_ros_msgs::msg::SpatialDetectionArray> cache_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

std::vector<SpatialDetectionSubscriber::DetectionPose> SpatialDetectionSubscriber::getLatestDetections()
{
  std::vector<SpatialDetectionSubscriber::DetectionPose> detection_poses;

  auto end_time = this->get_clock()->now();
  auto start_time = end_time - rclcpp::Duration(1s);
  auto messages = cache_.getInterval(start_time, end_time);

  //RCLCPP_INFO(LOGGER, "messages = %ld", messages.size());
  for(int i = messages.size() - 1; i >= 0; i--) {
    auto msg = messages[i];
    if (msg->detections.size() > 0) {
      RCLCPP_INFO(LOGGER, "header.stamp.sec = %d, detections = %ld", msg->header.stamp.sec, msg->detections.size());

      for (auto detection: msg->detections) {
        geometry_msgs::msg::PoseStamped pose_from_cam;
        pose_from_cam.header = msg->header;
        pose_from_cam.pose.position = detection.position;
        pose_from_cam.pose.position.y = - (pose_from_cam.pose.position.y);
        //pose_from_cam.pose.position.z -= 0.01; // add half width of the object

        RCLCPP_INFO(LOGGER, "pose_from_cam x: %0.1fcm, y: %0.1fcm, z: %0.1fcm",
            pose_from_cam.pose.position.x * 100.0,
            pose_from_cam.pose.position.y * 100.0,
            pose_from_cam.pose.position.z * 100.0);

        geometry_msgs::msg::PoseStamped pose_from_base_link =
                    tf_buffer_->transform(pose_from_cam, "base_link", tf2::Duration(0));

        DetectionPose detection_pose;
        detection_pose.class_id = detection.results[0].class_id;
        detection_pose.pose = pose_from_base_link.pose;
        detection_poses.push_back(detection_pose);
      }
      break;
    }
  }
  return detection_poses;
}

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(std::string object_id);

  void setupPlanningScene();

  void cleanPlanningScene();

  std::vector<std::string> 
	  updatePlanningScene(const std::vector<SpatialDetectionSubscriber::DetectionPose>& detections);

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(moveit_msgs::msg::CollisionObject object);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface psi;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  //moveit::planning_interface::PlanningSceneInterface psi;
  float angles[NUM_BINS] = { BIN_0_ANGLE, BIN_1_ANGLE };

  for (int i = 0; i < NUM_BINS; i++) {
    moveit_msgs::msg::CollisionObject bin;

    bin.id = "bin_" + std::to_string(i);
    bin.header.frame_id = "world";
    bin.primitives.resize(1);
    bin.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    bin.primitives[0].dimensions = { BIN_H, BIN_W, BIN_L };

    geometry_msgs::msg::Pose pose;
    pose.position.x = BIN_DIST * cos(angles[i]);
    pose.position.y = BIN_DIST * sin(angles[i]);
    pose.position.z = bin.primitives[0].dimensions[2] / 2.0f;  

    bin.pose = pose;

    psi.applyCollisionObject(bin);
  }
}

void MTCTaskNode::cleanPlanningScene()
{
  //moveit::planning_interface::PlanningSceneInterface psi;
 
  for (auto [object_id, object]: psi.getObjects(psi.getKnownObjectNames(true))) {
    RCLCPP_INFO(LOGGER, "cleanPlanningScene: object_type=%s", object.type.key.c_str());

    if (object.type.key == "0" || object.type.key == "1") {
      object.operation = object.REMOVE;
      psi.applyCollisionObject(object);
      RCLCPP_INFO(LOGGER, "cleanPlanningScene: removed %s", object_id.c_str());
    }
  }
}

std::vector<std::string>
  MTCTaskNode::updatePlanningScene(const std::vector<SpatialDetectionSubscriber::DetectionPose>& detections)
{
  //moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_ids;

  cleanPlanningScene();

  int idx = 0;

  for (auto detection: detections) {
    RCLCPP_INFO(LOGGER, "class: %s, x: %f, y: %f, z: %f",
                  detection.class_id.c_str(), detection.pose.position.x,
                  detection.pose.position.y, detection.pose.position.z);

    object_recognition_msgs::msg::ObjectType object_type;
    object_type.key = detection.class_id;

    moveit_msgs::msg::CollisionObject object;
    object.id = "object_" + std::to_string(idx++);
    object.type = object_type;
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;

    if (object.type.key == "0") {
      object.primitives[0].dimensions = { 0.04, 0.02, 0.06 };
    } else {
      object.primitives[0].dimensions = { 0.05, 0.03, 0.03 };
    } 

    // approximate to nearest integral angle (since generate pose has 1° angle delta)
    float r = sqrt(std::pow(detection.pose.position.x, 2) +  std::pow(detection.pose.position.y, 2));
    float radians = atan(detection.pose.position.y/detection.pose.position.x);
    float degrees = radians * 180.0f / M_PI;
    float degrees_1 = round(degrees);
    float angle = degrees_1 * M_PI / 180.0f;

    geometry_msgs::msg::Pose pose;
    pose.position.x = r * cos(angle);
    pose.position.y = r * sin(angle);
    //pose.position.z =  object.primitives[0].dimensions[2] / 2.0f;
    pose.position.z = detection.pose.position.z;
    object.pose = pose;

    RCLCPP_INFO(LOGGER, "Added object %s [r = %f, angle_0 = %f, angle_1 = %f] modified pose  x: %f, y: %f, z: %f",
                  object.id.c_str(), r, degrees, degrees_1, pose.position.x, pose.position.y, pose.position.z);

    object_ids.push_back(object.id);

    psi.applyCollisionObject(object);
  }

  return object_ids;
}

void MTCTaskNode::doTask(std::string object_id)
{
  //moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_ids {object_id};
  auto object = psi.getObjects(object_ids)[object_id];
  task_ = createTask(object);

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  object.operation = object.REMOVE;
  psi.applyCollisionObject(object);
  RCLCPP_INFO(LOGGER, "Removed %s",  object.id.c_str());
  rclcpp::sleep_for(100ms);

  return;
}

mtc::Task MTCTaskNode::createTask(moveit_msgs::msg::CollisionObject object)
{
  mtc::Task task;
  task.stages()->setName("pick_n_place task:" + object.id);
  task.loadRobotModel(node_);

  const auto& arm_group_name = "braccio_arm";
  //const auto& arm_group_name = "braccio_arm_gripper";
  const auto& hand_group_name = "braccio_gripper";
  const auto& hand_frame = "wrist_roll_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  //sampling_planner->setPlannerId("RRTConnectkConfigDefault");
  sampling_planner->setProperty("goal_joint_tolerance", 1e-3);
  sampling_planner->setProperty("goal_position_tolerance", 1e-3);
  sampling_planner->setProperty("goal_orientation_tolerance", 1e-3);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor (1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
  
 
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
  
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ 
        { arm_group_name, sampling_planner }});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;  
  float d = sqrt(std::pow(object.pose.position.x, 2) +  std::pow(object.pose.position.y, 2));
  //using curve fitting based on observed data
  float x_angle  = 3.85798 * d + (-0.21251);
  printf("d = %f, x_angle = %f\n", d, x_angle);
      
  // SerialContainer 
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef",  "group",  "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group" });
      stage->setMinMaxDistance(0.0, 0.07);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object.id);
      stage->setAngleDelta(M_PI / 180);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      //Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI/2 - M_PI/3.6, Eigen::Vector3d::UnitX()) *
      Eigen::Quaterniond q = Eigen::AngleAxisd(x_angle, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().y() = -0.01;
      grasp_frame_transform.translation().z() = 0.11;
      
      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef",  "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object.id,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object.id, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      //stage->setMinMaxDistance(0.06, 0.1);
      if (object.pose.position.z < 0.05) {
        stage->setMinMaxDistance(0.06, 0.1);
      } else {
        stage->setMinMaxDistance(0.0, 0.1);
      }
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }
 
  // Generate Place Pose
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
      
    float theta = 0.0f;

    if (object.type.key == "0") { 
      theta = BIN_0_ANGLE;
    } else if (object.type.key == "1") {
      theta = BIN_1_ANGLE;
    }

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");

      stage->setObject(object.id);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = object.id;
      target_pose_msg.pose.position.x = BIN_DIST * cos(theta) - object.pose.position.x;
      target_pose_msg.pose.position.y = BIN_DIST * sin(theta) - object.pose.position.y;
      target_pose_msg.pose.position.z = BIN_H + 0.04;

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
						      
      Eigen::Isometry3d place_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(M_PI,   Eigen::Vector3d::UnitY())
			   * Eigen::AngleAxisd((M_PI/2 - theta), Eigen::Vector3d::UnitZ());
       
      place_frame_transform.linear() = q.matrix();

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object.id,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object.id, hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      //stage->setMinMaxDistance(0.02, 0.05);
      stage->setMinMaxDistance(0.0, 0.05);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -BIN_DIST * cos(theta);
      vec.vector.y = -BIN_DIST * sin(theta);
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  auto spatial_sub_node = std::make_shared<SpatialDetectionSubscriber>();
  auto trigger_srv_node = std::make_shared<TriggerMotionPlanningService>();

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &mtc_task_node, &spatial_sub_node, &trigger_srv_node]() {
      executor.add_node(mtc_task_node->getNodeBaseInterface());
      executor.add_node(spatial_sub_node);
      executor.add_node(trigger_srv_node);
      executor.spin();
      executor.remove_node(trigger_srv_node);
      executor.remove_node(spatial_sub_node);
      executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  rclcpp::sleep_for(2s); // let the others listen to the publishers

  while (true) {
    if (trigger_srv_node->motion_planning_enabled) {
      auto detections = spatial_sub_node->getLatestDetections();
      //RCLCPP_INFO(LOGGER, "detections=%d", detections.size());

      if (detections.size() > 0) {
        auto object_ids = mtc_task_node->updatePlanningScene(detections);

        for (auto object_id: object_ids) {
          RCLCPP_INFO(LOGGER, "object id = %s", object_id.c_str());
          mtc_task_node->doTask(object_id);
        }
      }
    }

    rclcpp::sleep_for(100ms);
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
