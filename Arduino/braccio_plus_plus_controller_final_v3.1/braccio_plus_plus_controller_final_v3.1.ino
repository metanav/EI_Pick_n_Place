
#include <Braccio++.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <control_msgs/action/gripper_command.h>
#include <control_msgs/action/follow_joint_trajectory.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <hardware/pio.h>
#include "uart_tx.pio.h"
#include "WiFiNINA.h"

using namespace mbed;
using namespace rtos;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define TIMEOUT_MS 10
#define MAX_GOALS 1
#define ID_GRIPPER 1
#define QUEUE_SIZE 5

const int N_JOINTS =  SmartServoClass::NUM_MOTORS;

// micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_publisher_t joint_state_publisher;
rclc_action_server_t action_server_follow_joint_trajectory;
rclc_action_server_t action_server_gripper_command;

rcl_client_t service_client;
std_srvs__srv__Trigger_Request service_request;
std_srvs__srv__Trigger_Response service_response;
int64_t service_request_seq_num;

control_msgs__action__GripperCommand_SendGoal_Request ros_goal_request_gripper[MAX_GOALS];
control_msgs__action__FollowJointTrajectory_SendGoal_Request ros_goal_request_arm[MAX_GOALS];
sensor_msgs__msg__JointState joint_states_msg;

typedef enum { REQ_ARM, REQ_GRIPPER } req_type_t;
typedef struct {
  rclc_action_goal_handle_t *handle;
  req_type_t req_type; 
} goal_t;


MemoryPool<goal_t, QUEUE_SIZE> mpool;
Queue<goal_t, QUEUE_SIZE> queue;
Thread thread_ctrl;

// Braccio ++ joints
char joint_names[N_JOINTS][20] = {
  "base_joint",
  "shoulder_joint",
  "elbow_joint",
  "wrist_pitch_joint",
  "wrist_roll_joint",
  "gripper_joint"
};

const uint PIN_TX = 18;  // D6 on Nano RP2040 Connect (RX = pin 28 on RPi 4)
const uint SERIAL_BAUD = 115200;
PIO pio = pio0;
uint sm = 0;

void debug_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    uart_tx_program_puts(pio, sm, print_buf);
  }
}

void debug_printf_float(float f) {
  debug_printf("%f", f);
}
void leds_init()
{
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

void leds_off()
{
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, LOW);
}

void error_loop(rcl_ret_t temp_rc) {
  bool on = false;
  for (;;) {
    digitalWrite(LEDR, (on ? HIGH : LOW));
    on = !on;
    delay(250);
    debug_printf("Error: %d\n", temp_rc);
  }
}

void set_initial_position()
{
  float base_pos        = 0.0f; //RAD_TO_DEG * 1.48353f;
  float elbow_pos       = RAD_TO_DEG * 2.8f;
  float shoulder_pos    = RAD_TO_DEG * 2.8;
  float wrist_pitch_pos = RAD_TO_DEG * 4.2f;
  float wrist_roll_pos  = RAD_TO_DEG * 2.93215f;
  float grippper_pos    = RAD_TO_DEG * 2.79253f;
  Braccio.moveTo(grippper_pos, wrist_roll_pos, wrist_pitch_pos, elbow_pos, shoulder_pos, base_pos);
  delay(100);
}

void execute_gripper_command(control_msgs__msg__GripperCommand *command)
{
  float grippper_pos = RAD_TO_DEG * command->position;
  debug_printf("execute_gripper_command: %f\n", grippper_pos);
  auto gripper  = Braccio.get(ID_GRIPPER);
  gripper.move().to(grippper_pos);
  delay(500);
}

void execute_arm_trajectory(trajectory_msgs__msg__JointTrajectory *trajectory)
{
  //  MoveIt2 sends the joint_name in alphabetical order
  //  joint_names:
  //  - base_joint
  //  - elbow_joint
  //  - shoulder_joint
  //  - wrist_pitch_joint
  //  - wrist_roll_joint
  debug_printf("Begin trajectory execution\n");
  unsigned long int execution_start_ms = millis();

  for (int i = 0; i < trajectory->points.size; i++) {
    float base_pos        = RAD_TO_DEG * trajectory->points.data[i].positions.data[0];
    float elbow_pos       = RAD_TO_DEG * trajectory->points.data[i].positions.data[1];
    float shoulder_pos    = RAD_TO_DEG * trajectory->points.data[i].positions.data[2];
    float wrist_pitch_pos = RAD_TO_DEG * trajectory->points.data[i].positions.data[3];
    float wrist_roll_pos  = RAD_TO_DEG * trajectory->points.data[i].positions.data[4];
    float grippper_pos    = -1.0f; // set invalid angle so that it will be skipped  //Braccio.get(1).position();

    unsigned long int move_start_ms = millis();
    Braccio.moveTo(grippper_pos, wrist_roll_pos, wrist_pitch_pos, elbow_pos, shoulder_pos, base_pos);
    delay(50); 
    //debug_printf("moveTo took [%ld ms]\n", millis() - move_start_ms);
  }
  debug_printf("Trajectory executed in [%ld ms]\n", millis() - execution_start_ms);
}

void print_arm_trajectory(control_msgs__action__FollowJointTrajectory_SendGoal_Request *req)
{
  debug_printf("\nheader:\n");
  debug_printf("  stamp:\n");
  debug_printf("    sec: %ld\n", req->goal.trajectory.header.stamp.sec);
  debug_printf("    nanosec: %ld\n", req->goal.trajectory.header.stamp.nanosec);
  debug_printf("  frame_id: %s\n", req->goal.trajectory.header.frame_id.data);

  debug_printf("joint_names: ");
  for (int i = 0; i < req->goal.trajectory.joint_names.size; i++) {
    debug_printf("- %s\n", req->goal.trajectory.joint_names.data[i].data);
  }

  for (int i = 0; i < req->goal.trajectory.points.size; i++) {
    debug_printf("positions [%d] ", i);
    if (req->goal.trajectory.points.data[i].positions.size >  0) {
      debug_printf("\n");
      for (int j = 0; j < req->goal.trajectory.points.data[i].positions.size; j++) {
        debug_printf("  %0.4f\n", req->goal.trajectory.points.data[i].positions.data[j]);
      }
    } else {
      debug_printf("[]\n");
    }

    debug_printf("velocities [%d]: ", i);
    if (req->goal.trajectory.points.data[i].velocities.size >  0) {
      debug_printf("\n");
      for (int j = 0; j < req->goal.trajectory.points.data[i].velocities.size; j++) {
        debug_printf("  %0.4f\n", req->goal.trajectory.points.data[i].velocities.data[j]);
      }
    } else {
      debug_printf("[]\n");
    }

    debug_printf("accelerations [%d]: ", i);
    if (req->goal.trajectory.points.data[i].accelerations.size >  0) {
      debug_printf("\n");
      for (int j = 0; j < req->goal.trajectory.points.data[i].accelerations.size; j++) {
        debug_printf("  %0.4f\n", req->goal.trajectory.points.data[i].accelerations.data[j]);
      }
    } else {
      debug_printf("[]\n");
    }

    debug_printf("effort [%d]: ", i);
    if (req->goal.trajectory.points.data[i].effort.size >  0) {
      debug_printf("\n");
      for (int j = 0; j < req->goal.trajectory.points.data[i].effort.size; j++) {
        debug_printf("  %0.4f\n", req->goal.trajectory.points.data[i].effort.data[j]);
      }
    } else {
      debug_printf("[]\n");
    }

    debug_printf("time_from_start:\n");
    debug_printf("  sec: %ld\n", req->goal.trajectory.points.data[i].time_from_start.sec);
    debug_printf("  nanosec: %ld\n", req->goal.trajectory.points.data[i].time_from_start.nanosec);
  }

  debug_printf("Total trajectory points = %ld\n", req->goal.trajectory.points.size);
}

void gripper_control(rclc_action_goal_handle_t *goal_handle)
{
   rcl_action_goal_state_t goal_state;

   control_msgs__action__GripperCommand_SendGoal_Request * req =
     (control_msgs__action__GripperCommand_SendGoal_Request *) goal_handle->ros_goal_request;

   control_msgs__action__GripperCommand_SendGoal_Response response = {0};
   control_msgs__action__GripperCommand_FeedbackMessage feedback;

   if (req->goal.command.max_effort > 5.0f) {
     goal_state = GOAL_STATE_ABORTED;
     digitalWrite(LEDR, HIGH);
   } else {
     if (!goal_handle->goal_cancelled) {
       goal_state = GOAL_STATE_SUCCEEDED;
       digitalWrite(LEDG, HIGH);
       execute_gripper_command(&req->goal.command);
     } else {
       goal_state = GOAL_STATE_CANCELED;
       digitalWrite(LEDB, HIGH);
     }
   }

   rcl_ret_t rc;
   do {
     rc = rclc_action_send_result(goal_handle, goal_state, &response);
     delay(500);
   } while (rc != RCL_RET_OK);
      
   leds_off();
}

void arm_control(rclc_action_goal_handle_t *goal_handle)
{
  rcl_action_goal_state_t goal_state;

  control_msgs__action__FollowJointTrajectory_SendGoal_Request * req =
    (control_msgs__action__FollowJointTrajectory_SendGoal_Request *) goal_handle->ros_goal_request;

  control_msgs__action__FollowJointTrajectory_SendGoal_Response response = {0};
  control_msgs__action__FollowJointTrajectory_FeedbackMessage feedback;

  //print_arm_trajectory(req);

  if (req->goal.trajectory.header.frame_id.size = 0) {
    goal_state = GOAL_STATE_ABORTED;
    digitalWrite(LEDR, HIGH);
  } else {
    if (!goal_handle->goal_cancelled) {
      goal_state = GOAL_STATE_SUCCEEDED;
      digitalWrite(LEDG, HIGH);
      execute_arm_trajectory(&req->goal.trajectory);
    } else {
      goal_state = GOAL_STATE_CANCELED;
      digitalWrite(LEDB, HIGH);
    }
  }

  rcl_ret_t rc;
  do {
    rc = rclc_action_send_result(goal_handle, goal_state, &response);
    delay(500);
    debug_printf("goal_handler_thread: rc =  %d\n", rc);
  } while (rc != RCL_RET_OK);
  
  leds_off();
}

void controller_thread()
{
  while (true) {
    osEvent evt = queue.get();

    if (evt.status == osEventMessage) {
      goal_t *goal = (goal_t*)evt.value.p;

      if (goal->req_type == REQ_ARM) {
        debug_printf("%s: arm goal_handle received.\n", __FUNCTION__);
        arm_control(goal->handle);
      } else if (goal->req_type == REQ_GRIPPER) {
        debug_printf("%s: gripper goal_handle received.\n", __FUNCTION__);
        gripper_control(goal->handle);
      } else {
        debug_printf("%s: req_type [%d] not handled.\n", __FUNCTION__, goal->req_type);
      }

      mpool.free(goal);
    }

    ThisThread::sleep_for(50ms);
  }
}

rcl_ret_t gripper_goal_callback(rclc_action_goal_handle_t *goal_handle, void *context)
{
  (void) context;
  debug_printf("gripper_goal_callback\n");

  control_msgs__action__GripperCommand_SendGoal_Request * req =
    (control_msgs__action__GripperCommand_SendGoal_Request *) goal_handle->ros_goal_request;

  // TODO: add GripperCommand request validation here

  goal_t *goal = mpool.alloc();; 

  if (goal == NULL) {
    debug_printf("%s: mpool allocation failed.\n", __FUNCTION__);
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  goal->handle = goal_handle;
  goal->req_type = REQ_GRIPPER;
  queue.put(goal);

  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

rcl_ret_t arm_goal_callback(rclc_action_goal_handle_t *goal_handle, void *context)
{
  (void) context;
  debug_printf("arm_goal_callback\n");

  control_msgs__action__FollowJointTrajectory_SendGoal_Request * req =
    (control_msgs__action__FollowJointTrajectory_SendGoal_Request *) goal_handle->ros_goal_request;

  // TODO: add FollowJointTrajectory request validation here

  goal_t *goal = mpool.alloc();; 

  if (goal == NULL) {
    debug_printf("%s: mpool allocation failed.\n", __FUNCTION__);
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  goal->handle = goal_handle;
  goal->req_type = REQ_ARM;
  queue.put(goal);

  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool gripper_cancel_callback(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  debug_printf("gripper_cancel_callback");

  return true;
}

bool arm_cancel_callback(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  debug_printf("arm_cancel_callback");

  return true;
}

void create_joint_states_message_memory()
{
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = N_JOINTS;
  conf.max_basic_type_sequence_capacity = N_JOINTS;

  bool success = micro_ros_utilities_create_message_memory(
                   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                   &joint_states_msg,
                   conf
                 );
  micro_ros_string_utilities_set(joint_states_msg.header.frame_id, "braccio_plus_plus_joint_states");

  for (int i = 0; i < N_JOINTS; i++) {
    micro_ros_string_utilities_set(joint_states_msg.name.data[i], joint_names[i]);
  }
  joint_states_msg.name.size = N_JOINTS;
}

void create_joint_trajectory_request_message_memory()
{
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity =  1;
  conf.max_basic_type_sequence_capacity = N_JOINTS;

  micro_ros_utilities_memory_rule_t rules[] = {
    {"goal.trajectory.joint_names", N_JOINTS},
    {"goal.trajectory.points", 55},
    {"goal.trajectory.points.positions", N_JOINTS},
    {"goal.trajectory.points.velocities", N_JOINTS},
    {"goal.trajectory.points.accelerations", N_JOINTS},
  };
  conf.rules = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);

  size_t dynamic_size = micro_ros_utilities_get_dynamic_size(
                          ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request),
                          conf
                        );
  size_t message_total_size = dynamic_size + sizeof(control_msgs__action__FollowJointTrajectory_SendGoal_Request);

  debug_printf("Message Size = %ld\n", message_total_size);

  for (int i = 0; i < MAX_GOALS; i++) {
    bool success = micro_ros_utilities_create_message_memory(
                     ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, action, FollowJointTrajectory_SendGoal_Request),
                     &ros_goal_request_arm[i],
                     conf
                   );

    debug_printf("micro_ros_utilities_create_message_memory returns:%d\n", success);
  }
}

void create_service_response_message_memory()
{
  service_response.message.data = (char *) malloc(100);
  service_response.message.size = 0;
  service_response.message.capacity = 100;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    int64_t time_ms = rmw_uros_epoch_millis() / 1000;
    int64_t time_ns = rmw_uros_epoch_nanos();

    joint_states_msg.header.stamp.sec = time_ms;
    joint_states_msg.header.stamp.nanosec = time_ns;

    float current_servo_pos[N_JOINTS] = {0};
    Braccio.positions(current_servo_pos);
    std::reverse(current_servo_pos, current_servo_pos + N_JOINTS);

    bool invalid_angle = false;
    for (size_t i = 0; i < N_JOINTS; i++) {
      if (current_servo_pos[i] > SmartServoClass::MAX_ANGLE) {
        invalid_angle = true;
        //debug_printf("joint [%d] invalid angle: %f\n", i, current_servo_pos[i]);

        break;
      }
      joint_states_msg.position.data[i] =  DEG_TO_RAD * current_servo_pos[i];
      joint_states_msg.velocity.data[i] = 0.0f;
      joint_states_msg.effort.data[i] = 0.0f;
    }

    if (!invalid_angle) {
      joint_states_msg.position.size = N_JOINTS;
      joint_states_msg.velocity.size = N_JOINTS;
      joint_states_msg.effort.size = N_JOINTS;

      rcl_publish(&joint_state_publisher, &joint_states_msg, NULL);
    }
  }
}

void service_client_callback(const void * s_response)
{
  std_srvs__srv__Trigger_Response * s_response_in = (std_srvs__srv__Trigger_Response *) s_response;
  debug_printf("service_client_callback: %d\n", s_response_in->success);
}

void setup()
{
  // Braccio++ carrier board MCU (RP2040) has 2 hardware UART which are already in use by Braccio++ RS485 and WifiNina
  // UartUSB is used by microROS agent to communicate. So we are adding a PIO Uart tx only to print debug messages.
  uint offset = pio_add_program(pio, &uart_tx_program);
  uart_tx_program_init(pio, sm, offset, PIN_TX, SERIAL_BAUD);
  debug_printf("Debug messages via PIO TX\n");

  set_microros_transports();

  leds_init();
  leds_off();

  delay(500);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  debug_printf("rclc_support_init\n");

  // create node
  RCCHECK(rclc_node_init_default(&node, "braccio_plus_plus_node", "", &support));
  debug_printf("rclc_node_init_default\n");

  // Create gripper command action service
  RCCHECK(
    rclc_action_server_init_default(
      &action_server_gripper_command,
      &node,
      &support,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(control_msgs, GripperCommand),
      "/gripper/gripper_cmd"
    ));
  debug_printf("action_server_gripper_command\n");

  // Create follow joint trajectory action service
  RCCHECK(
    rclc_action_server_init_default(
      &action_server_follow_joint_trajectory,
      &node,
      &support,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(control_msgs, FollowJointTrajectory),
      "/arm/follow_joint_trajectory"
    ));
  debug_printf("action_server_follow_joint_trajectory\n");

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &joint_state_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/joint_states"));
  debug_printf("rclc_publisher_init_default\n");

  //   create timer
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(200),
            timer_callback));
  debug_printf("rclc_timer_init_default");

  // create service client
  RCCHECK(rclc_client_init_default(
            &service_client,
            &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
            "/trigger_motion_planning"));
  debug_printf("rclc_client_init_default");


  // Creating messages memory
  create_joint_states_message_memory();
  debug_printf("create_joint_states_message_memory: OK\n");
  create_joint_trajectory_request_message_memory();
  debug_printf("create_joint_trajectory_request_message_memory: OK\n");
  create_service_response_message_memory();
  debug_printf("create_service_response_message_memory: OK\n");

  // create executor
  const unsigned int num_handles = 4;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  debug_printf("rclc_executor_init\n");

  rclc_executor_add_timer(&executor, &timer);

  RCCHECK(
    rclc_executor_add_action_server(
      &executor,
      &action_server_gripper_command,
      MAX_GOALS,
      ros_goal_request_gripper,
      sizeof(control_msgs__action__GripperCommand_SendGoal_Request),
      gripper_goal_callback,
      gripper_cancel_callback,
      (void *) &action_server_gripper_command));

  debug_printf("action_server_gripper_command\n");

  RCCHECK(
    rclc_executor_add_action_server(
      &executor,
      &action_server_follow_joint_trajectory,
      MAX_GOALS,
      ros_goal_request_arm,
      sizeof(control_msgs__action__FollowJointTrajectory_SendGoal_Request),
      arm_goal_callback,
      arm_cancel_callback,
      (void *) &action_server_follow_joint_trajectory));

  debug_printf("action_server_follow_joint_trajectory\n");

  RCCHECK(rclc_executor_add_client(
            &executor,
            &service_client,
            &service_response,
            &service_client_callback));

  debug_printf("service_client\n");

  thread_ctrl.start(controller_thread);
  debug_printf("thread started\n");

  if (!Braccio.begin()) {
    debug_printf("Error: Braccio.begin failed.\n");
    error_loop(0);
  }
  
  Braccio.speed(SLOW);
  //Braccio.setMaxTorque(500); // max torque upper limit = 1000
  set_initial_position();
  debug_printf("set arm to initial position\n");

  // sync time
  RCCHECK(rmw_uros_sync_session(TIMEOUT_MS));
  debug_printf("synced time\n");
}

// for button debounce
unsigned long last_trigger_ms = 0;

void loop()
{
  delay(50);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  if (Braccio.isButtonPressed_ENTER() && (millis() - last_trigger_ms > 1000)) {
    // trigger motion planning
    std_srvs__srv__Trigger_Request__init(&service_request);
    service_request.structure_needs_at_least_one_member = 0;
    RCCHECK(rcl_send_request(
              &service_client,
              &service_request,
              &service_request_seq_num));
    last_trigger_ms = millis();
  }
}
