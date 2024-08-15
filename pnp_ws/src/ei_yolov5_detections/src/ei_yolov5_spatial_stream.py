import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
from socketserver import ThreadingMixIn
from time import sleep
import depthai as dai
import numpy as np
import cv2
from PIL import Image
import blobconverter

HTTP_SERVER_PORT = 8090

class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
        self.end_headers()
        while True:
            sleep(0.1)
            if hasattr(self.server, 'frametosend'):
                ok, encoded = cv2.imencode('.jpg', self.server.frametosend)
                self.wfile.write("--jpgboundary".encode())
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)
                self.end_headers()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    pass


# start MJPEG HTTP Server
server_HTTP = ThreadedHTTPServer(('0.0.0.0', HTTP_SERVER_PORT), VideoStreamHandler)
t = threading.Thread(target=server_HTTP.serve_forever)
t.daemon = True
t.start()

labelMap = [ "Penguin", "Pig" ]

syncNN = True
BLOB_PATH = '~/pnp_ws/src/ei_yolov5_detections/resources/ei-pnp_yolov5n_320_openvino_2022.1_6shave.blob'

def create_pipeline(depth):
    pipeline = dai.Pipeline()
    colorCam = pipeline.create(dai.node.ColorCamera)

    spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    colorCam.setPreviewSize(320, 320)
    colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    colorCam.setInterleaved(False)
    colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    spatialDetectionNetwork.setBlobPath(BLOB_PATH)
    spatialDetectionNetwork.setConfidenceThreshold(0.7)
    spatialDetectionNetwork.input.setBlocking(False)

    # YOLO specific parameters
    spatialDetectionNetwork.setNumClasses(2)
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]);
    spatialDetectionNetwork.setAnchorMasks({
        "side40": [0, 1, 2],
        "side20": [3, 4, 5],
        "side10": [6, 7, 8] })
    spatialDetectionNetwork.setIouThreshold(0.5)

    if depth:
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Setting node configs
        stereo.initialConfig.setConfidenceThreshold(200)
        stereo.setRectifyEdgeFillColor(0)
        stereo.initialConfig.setLeftRightCheckThreshold(5);
        stereo.setSubpixel(False);
        stereo.setExtendedDisparity(True);
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.25)
        spatialDetectionNetwork.setDepthLowerThreshold(200)
        spatialDetectionNetwork.setDepthUpperThreshold(1000)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    colorCam.preview.link(spatialDetectionNetwork.input)
    if syncNN:
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)
    else:
        colorCam.preview.link(xoutRgb.input)


    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutNN.setStreamName("detections")
    spatialDetectionNetwork.out.link(xoutNN.input)

    return pipeline

# Pipeline is defined, now we can connect to the device
with dai.Device() as device:
    cams = device.getConnectedCameras()
    depth_enabled = True

    # Start pipeline
    device.startPipeline(create_pipeline(depth_enabled))

    print(f"DepthAI is up & running. Navigate to '0.0.0.0:{str(HTTP_SERVER_PORT)}' with Chrome to see the mjpeg stream")

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    if depth_enabled:
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    frame = None
    depthFrame = None
    detections = []

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    while True:
        inPreview = previewQueue.get()
        frame = inPreview.getCvFrame()

        inNN = detectionNNQueue.get()
        detections = inNN.detections

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        if depth_enabled:
            depthFrame = depthQueue.get().getFrame()

            depthFrame = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrame = cv2.equalizeHist(depthFrame)
            depthFrame = cv2.applyColorMap(depthFrame, cv2.COLORMAP_HOT)

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        for detection in detections:
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            if depth_enabled:
                cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

            if depthFrame is not None:
                roi = detection.boundingBoxMapping.roi
                roi = roi.denormalize(depthFrame.shape[1], depthFrame.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)
                cv2.rectangle(depthFrame, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)


        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        if depth_enabled:
            new_width = int(depthFrame.shape[1] * (frame.shape[0] / depthFrame.shape[0]))
            stacked = np.hstack([frame, cv2.resize(depthFrame, (new_width, frame.shape[0]))])
            cv2.imshow("stacked", stacked)
            server_HTTP.frametosend = stacked
        else:
            cv2.imshow("frame", frame)
            server_HTTP.frametosend = frame


        if cv2.waitKey(1) == ord('q'):
            break
