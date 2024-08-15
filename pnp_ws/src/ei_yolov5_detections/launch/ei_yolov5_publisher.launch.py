import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    package_path = get_package_share_directory('ei_yolov5_detections')
    default_resources_path = os.path.join(package_path, 'resources')

    tf_prefix          = LaunchConfiguration('tf_prefix',     default = 'oak')
    camera_param_uri   = LaunchConfiguration('camera_param_uri',   default = 'package://depthai_examples/params/camera')
    sync_nn            = LaunchConfiguration('sync_nn',            default = True)
    subpixel           = LaunchConfiguration('subpixel',           default = True)
    nnName             = LaunchConfiguration('nnName',             default = "x")
    resourceBaseDir    = LaunchConfiguration('resourceBaseDir',    default = default_resources_path)
    confidence         = LaunchConfiguration('confidence',         default = 200)
    lrCheckTresh       = LaunchConfiguration('lrCheckTresh',       default = 5)
    monoResolution     = LaunchConfiguration('monoResolution',     default = '400p')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_camera_param_uri_cmd = DeclareLaunchArgument(
        'camera_param_uri',
        default_value=camera_param_uri,
        description='Sending camera yaml path')

    declare_sync_nn_cmd = DeclareLaunchArgument(
        'sync_nn',
        default_value=sync_nn,
        description='Syncs the image output with the Detection.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='Enables subpixel stereo detection.')

    declare_nnName_cmd = DeclareLaunchArgument(
        'nnName',
        default_value=nnName,
        description='Path to the object detection blob needed for detection')
    
    declare_resourceBaseDir_cmd = DeclareLaunchArgument(
        'resourceBaseDir',
        default_value=resourceBaseDir,
        description='Path to the resources directory which contains the default blobs for the network')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='Confidence that the disparity from the feature matching was good. 0-255. 255 being the lowest confidence.')
    
    declare_lrCheckTresh_cmd = DeclareLaunchArgument(
        'lrCheckTresh',
        default_value=lrCheckTresh,
        description='LR Threshold is the threshod of how much off the disparity on the l->r and r->l  ')

    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')
  
    ei_yolov5_spatial_node = launch_ros.actions.Node(
            package='ei_yolov5_detections', executable='ei_yolov5_spatial_node',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'camera_param_uri': camera_param_uri},
                        {'sync_nn': sync_nn},
                        {'nnName': nnName},
                        {'resourceBaseDir': resourceBaseDir},
                        {'monoResolution': monoResolution}])

    ei_yolov5_bbox_image_node = launch_ros.actions.Node(
            package='ei_yolov5_detections', executable='ei_yolov5_bbox_image_node',
            output='screen')

    return LaunchDescription([
        declare_tf_prefix_cmd,
        declare_camera_param_uri_cmd,
        declare_sync_nn_cmd,
        declare_subpixel_cmd,
        declare_nnName_cmd,
        declare_resourceBaseDir_cmd,
        declare_confidence_cmd,
        declare_lrCheckTresh_cmd,
        declare_monoResolution_cmd,
        ei_yolov5_spatial_node,
        ei_yolov5_bbox_image_node,
    ])

