import os
from launch import LaunchDescription, launch_description_sources
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace    = LaunchConfiguration('namespace',     default = '')
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    cam_pos_x = LaunchConfiguration('cam_pos_x',  default = '0.0')
    cam_pos_y = LaunchConfiguration('cam_pos_y',  default = '0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z',  default = '0.0')
    cam_roll  = LaunchConfiguration('cam_roll',   default = '0.0')
    cam_pitch = LaunchConfiguration('cam_pitch',  default = '0.0') 
    cam_yaw   = LaunchConfiguration('cam_yaw',    default = '0.0')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=namespace,
        description='Specifies the namespace of the robot state publisher node.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    urdf_launch = IncludeLaunchDescription(
      launch_description_sources.PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('pick_n_place'), 'launch', 'urdf_launch.py')),
      launch_arguments={
        'parent_frame': parent_frame,
        'cam_pos_x'   : cam_pos_x,
        'cam_pos_y'   : cam_pos_y,
        'cam_pos_z'   : cam_pos_z,
        'cam_roll'    : cam_roll,
        'cam_pitch'   : cam_pitch,
        'cam_yaw'     : cam_yaw}.items()
    )

    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("braccio")
        .robot_description(file_path="config/braccio.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            { "capabilities": "move_group/ExecuteTaskSolutionCapability" }
        ],
    )

    # MTC node
    pick_n_place_node = Node(
        package="pick_n_place",
        executable="pick_n_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_tf_prefix_cmd,
        declare_camera_model_cmd,
        declare_base_frame_cmd,
        declare_parent_frame_cmd,
        declare_pos_x_cmd,
        declare_pos_y_cmd,
        declare_pos_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        urdf_launch,
        run_move_group_node,
        pick_n_place_node
    ])

