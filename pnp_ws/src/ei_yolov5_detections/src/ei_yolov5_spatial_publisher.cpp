
#include <cstdio>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <sensor_msgs/msg/image.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#define NN_IMG_WIDTH  320
#define NN_IMG_HEIGHT 320

const std::vector<std::string> label_map = { "Penguin", "Pig" };

dai::Pipeline createPipeline(bool syncNN, bool subpixel, std::string nnPath, int confidence, int LRchecktresh, std::string resolution) 
{
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(NN_IMG_WIDTH, NN_IMG_HEIGHT);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B); // Left
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C); // Right

    /// setting node configs
    stereo->initialConfig.setConfidenceThreshold(confidence);
    //stereo->initialConfig.setDisparityShift(30);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(false);
    stereo->setExtendedDisparity(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A); // RGB
    auto config = stereo->initialConfig.get();
    config.postProcessing.thresholdFilter.minRange = 250;
    config.postProcessing.thresholdFilter.maxRange = 750;
    stereo->initialConfig.set(config);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.70f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.25);
    spatialDetectionNetwork->setDepthLowerThreshold(250);
    spatialDetectionNetwork->setDepthUpperThreshold(750);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(2);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326});
    spatialDetectionNetwork->setAnchorMasks({ {"side40", {0, 1, 2}}, {"side20", {3, 4, 5}}, {"side10", {6, 7, 8}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    if (syncNN) {
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    } else {
        colorCam->preview.link(xoutRgb->input);
    }

    spatialDetectionNetwork->out.link(xoutNN->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    return pipeline;
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ei_yolov5_spatial_node");

    std::string tfPrefix, resourceBaseDir, nnPath;
    std::string camera_param_uri;
    std::string nnName(BLOB_NAME);  // Set your blob name for the model here
    bool syncNN, subpixel;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseDir", "");

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseDir", resourceBaseDir);

    if(resourceBaseDir.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseDir\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseDir + "/" + nnName;
    std::cout << nnPath.c_str() << std::endl;
    dai::Pipeline pipeline = createPipeline(syncNN, subpixel, nnPath, confidence, LRchecktresh, monoResolution);
    

    dai::Device device(pipeline);

    device.setTimesync(true);

    std::cout <<  "USB Speed: " << device.getUsbSpeed() << std::endl;

    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto detectionQueue = device.getOutputQueue("detections", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto calibrationHandler = device.readCalibration();

    int width = 640;
    int height = 400;

    auto boardName = calibrationHandler.getEepromData().boardName;

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
        node,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");

    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 
		    NN_IMG_WIDTH, NN_IMG_HEIGHT, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        node,
        std::string("/ei_yolov5/spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        depthQueue,
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "stereo");

    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.
    depthPublish.addPublisherCallback();

    detectionPublish.addPublisherCallback();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
