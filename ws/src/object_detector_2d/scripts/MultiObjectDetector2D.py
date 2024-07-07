#!/usr/bin/env python3
# Published detections in 2D and 3D with a specified model.

import numpy as np
import argparse
import torch
#import tensorflow as tf
import cv2
import pathlib
import rospy
import threading
import imutils
import time
from imutils.video import FPS
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from frida_manipulation_interfaces.msg import objectDetection, objectDetectionArray
import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *
import tf2_ros
import tf2_geometry_msgs
import imutils
import copy
from ultralytics import YOLO

SOURCES = {
    "VIDEO": str(pathlib.Path(__file__).parent) + "/../resources/test.mp4",
    "CAMERA": 0,
    "ROS_IMG": "/camaras/0",
}

ARGS= {
    "SOURCE": SOURCES["VIDEO"],
    "ROS_INPUT": False,
    "USE_ACTIVE_FLAG": True,
    "DEPTH_ACTIVE": False,
    "DEPTH_INPUT": "/camera/depth/image_raw",
    "CAMERA_INFO": "/camera/depth/camera_info",
    "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/",
    "YOLO5_MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/yolov5n.pt",
    "YOLO8_MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/yolov8n.pt",
    "LABELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/label_map.pbtxt",
    "MIN_SCORE_THRESH": 0.75,
    "VERBOSE": False,
    "CAMERA_FRAME": "xtion_rgb_optical_frame",
    "USE_YOLO8": False,
    "YOLO_MODEL_PATH": str(pathlib.Path(__file__).parent) + "/../models/yolov5s.pt",
    "FLIP_IMAGE": False,
    "DETECTIONS_TOPIC": "/detections",
    "DETECTIONS_IMAGE_TOPIC": "/detections_image",
    "3D_MARKER_PUBLISHER": "/detections_3d_markers",
}

class CamaraProcessing:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.imageInfo = CameraInfo()
        
        # ARGS["MODELS_PATH"] = ARGS["MODELS_PATH"] is a python list but written as a string, transform it to the list of strings
        # e.g "['model1','model2']" -> list: ['model1', 'model2']
        self.YOLO5_MODELS_PATHS = ARGS["YOLO5_MODELS_PATH"]
        self.YOLO8_MODELS_PATHS = ARGS["YOLO8_MODELS_PATH"]
        self.YOLO5_MODELS_PATHS = self.YOLO5_MODELS_PATHS[1:-1].replace("'", "").split(",") if self.YOLO5_MODELS_PATHS != "" else []
        self.YOLO8_MODELS_PATHS = self.YOLO8_MODELS_PATHS[1:-1].replace("'", "").split(",") if self.YOLO8_MODELS_PATHS != "" else []
        
        rospy.loginfo(f"[INFO] YOLOv5 models to load: {self.YOLO5_MODELS_PATHS}")
        rospy.loginfo(f"[INFO] YOLOv8 models to load: {self.YOLO8_MODELS_PATHS}")
        rospy.loginfo("[INFO] Loading models...")
        
        def loadYolov5Model(model_path):
            model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=False)
            # color should be used for drawing bboxes
            self.yolov5_models.append({"model": model, "path": model_path, "color": np.random.randint(0, 255, 3)})

        def yolov8_warmup(model, repetitions=1, verbose=False):
            # Warmup model
            startTime = time.time()
            # create an empty frame to warmup the model
            for i in range(repetitions):
                warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
                model.predict(source=warmupFrame, verbose=verbose)
            rospy.loginfo(f"Model warmed up in {time.time() - startTime} seconds")

        def loadYolov8Model(model_path):
            model = YOLO(model_path)
            yolov8_warmup(model, repetitions=5, verbose=False)
            # generates random rgb colors for each model's bboxes
            self.yolov8_models.append({"model": model, "path": model_path, "color": np.random.randint(0, 255, 3)})

        self.yolov5_models = []
        self.yolov8_models = []
        for model_path in self.YOLO5_MODELS_PATHS:
            if model_path != "":
                rospy.loginfo(f"[INFO] Loading YOLOv5 model from {model_path}")
                loadYolov5Model(model_path)
        for model_path in self.YOLO8_MODELS_PATHS:
            if model_path != "":
                rospy.loginfo(f"[INFO] Loading YOLOv8 model from {model_path}")
                loadYolov8Model(model_path)
        
        rospy.loginfo("[INFO] Models loaded")
        
        self.activeFlag = not ARGS["USE_ACTIVE_FLAG"]
        self.runThread = None
        self.subscriber = None
        self.handleSource()
        self.publisher = rospy.Publisher(ARGS["DETECTIONS_TOPIC"], objectDetectionArray, queue_size=5)
        self.image_publisher = rospy.Publisher(ARGS["DETECTIONS_IMAGE_TOPIC"], Image, queue_size=5)
        # to visualize the 3d points of detected objects
        self.objects_publisher_3d = rospy.Publisher(ARGS["3D_MARKER_PUBLISHER"], MarkerArray, queue_size=5)

        # TFs
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        if ARGS["USE_ACTIVE_FLAG"]:
            rospy.Subscriber('detectionsActive', Bool, self.activeFlagSubscriber)

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

        # Show OpenCV window.
        try:
            self.detections_frame = []
            rate = rospy.Rate(60)
            while not rospy.is_shutdown():
                if ARGS["VERBOSE"] and len(self.detections_frame) != 0:
                    cv2.imshow("Detections", self.detections_frame)
                    cv2.waitKey(1)

                if len(self.detections_frame) != 0:
                    self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.detections_frame, "bgr8"))
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cv2.destroyAllWindows()
    
    # Callback for active flag
    def activeFlagSubscriber(self, msg):
        self.activeFlag = msg.data

    # Function to handle either a cv2 image or a ROS image.
    def handleSource(self):
        if ARGS["ROS_INPUT"]:
            self.subscriber = rospy.Subscriber(ARGS["SOURCE"], Image, self.imageRosCallback)
            if ARGS["DEPTH_ACTIVE"]:
                self.subscriberDepth = rospy.Subscriber(ARGS["DEPTH_INPUT"], Image, self.depthImageRosCallback)
                self.subscriberInfo = rospy.Subscriber(ARGS["CAMERA_INFO"], CameraInfo, self.infoImageRosCallback)
        else:
            cThread = threading.Thread(target=self.cameraThread, daemon=True)
            cThread.start()

    # Function to handle a cv2 input.
    def cameraThread(self):
        cap = cv2.VideoCapture(ARGS["SOURCE"])
        frame = []
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    continue
                if len(frame) == 0:
                    continue
                self.imageCallback(frame)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cap.release()

    # Function to handle a ROS input.
    def imageRosCallback(self, data):
        try:
            self.imageCallback(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
    
    # Function to handle ROS camera info input.
    def infoImageRosCallback(self, data):
        self.imageInfo = data
        self.subscriberInfo.unregister()

    # Function to handle Rate Neckbottle, TF object detection model frame rate (<10FPS) against camera input (>30FPS).
    # Process a frame only when the script finishes the process of the previous frame, rejecting frames to keep real-time idea.
    def imageCallback(self, img):
        self.rgb_image = img
        if not self.activeFlag:
            self.detections_frame = img
        elif self.runThread == None or not self.runThread.is_alive():
            self.runThread = threading.Thread(target=self.run, args=(img, ), daemon=True)
            self.runThread.start()

    # Handle FPS calculation.
    def callFps(self):	
        if self.fps != None:
            self.fps.stop()
            if ARGS["VERBOSE"]:
                print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
            self.fpsValue = self.fps.fps()

        self.fps = FPS().start()
        
        callFpsThread = threading.Timer(2.0, self.callFps, args=())
        callFpsThread.start()

    def yolov5_run_inference_on_image(self, model, frame):
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model["model"](frame)
        height = frame.shape[1]
        width = frame.shape[0]
        
        for *xyxy, conf, cls,names in results.pandas().xyxy[0].itertuples(index=False):
            # Normalized [0-1] ymin, xmin, ymax, xmax
            if conf < ARGS["MIN_SCORE_THRESH"]:
                continue
            
            self.visual_detections['detection_boxes'].append([xyxy[0]/height, xyxy[1]/width, xyxy[2]/height, xyxy[3]/width])
            self.visual_detections['detection_classes'].append(cls)
            self.visual_detections['detection_names'].append(names)
            self.visual_detections['detection_scores'].append(conf)
            self.visual_detections['detection_colors'].append(model["color"])
            
    def yolov8_run_inference_on_image(self, model, frame):
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model["model"](frame)
        
        height = frame.shape[0]
        width = frame.shape[1]
        
        boxes = []
        confidences = []
        classids = []
        names = []

        for out in results:
                for box in out.boxes:
                    prob = round(box.conf[0].item(), 2)
                    if prob < ARGS["MIN_SCORE_THRESH"]:
                        continue
                    
                    x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                    x1 = x1/width
                    x2 = x2/width
                    y1 = y1/height
                    y2 = y2/height

                    class_id = box.cls[0].item()

                    boxes.append([x1, y1, x2, y2])
                    confidences.append(float(prob))
                    classids.append(class_id)
                    names.append(model["model"].model.names[class_id])
                    # print(f"Found {class_id} in {x1} {y1} {x2} {y2}")
                    # print("------------------------------")
                    # print()
        for i in range(len(boxes)):
            self.visual_detections['detection_boxes'].append(boxes[i])
            self.visual_detections['detection_classes'].append(classids[i])
            self.visual_detections['detection_names'].append(names[i])
            self.visual_detections['detection_scores'].append(confidences[i])
            self.visual_detections['detection_colors'].append(model["color"])

    # Handle the detection model input/output.
    def compute_result(self, frame):
        visual_frame = copy.deepcopy(frame)
        if ARGS["FLIP_IMAGE"]:
            visual_frame = imutils.rotate(frame, 180)
        
        self.visual_detections = {
            "detection_boxes": [],
            "detection_classes": [],
            "detection_names": [],
            "detection_scores": [],
            "detection_colors": [],
        }
        
        for model_dict in self.yolov5_models:
            print(f"Running inference on YOLOv5 model {model_dict['path']}")
            self.yolov5_run_inference_on_image(model_dict, visual_frame)
        for model_dict in self.yolov8_models:
            print(f"Running inference on YOLOv8 model {model_dict['path']}")
            self.yolov8_run_inference_on_image(model_dict, visual_frame)
        
        # transform all to numpy arrays
        for key in self.visual_detections:
            self.visual_detections[key] = np.array(self.visual_detections[key])
        
        detections = copy.deepcopy(self.visual_detections)
        detections["detection_boxes"] = np.array([[1 - box[2], 1 - box[3], 1 - box[0], 1 - box[1]] for box in self.visual_detections["detection_boxes"]]) if ARGS["FLIP_IMAGE"] else self.visual_detections["detection_boxes"]
        return self.get_objects(detections["detection_boxes"],
                                detections["detection_scores"],
                                detections["detection_classes"],
                                detections["detection_names"],
                                frame.shape[0],
                                frame.shape[1],
                                frame), self.visual_detections, visual_frame

    # This function creates the output array of the detected objects with its 2D & 3D coordinates.
    def get_objects(self, boxes, scores, classes, names, height, width, frame):
        objects = {}
        res = []

        pa = PoseArray()
        pa.header.frame_id = ARGS["CAMERA_FRAME"]
        pa.header.stamp = rospy.Time.now()

        for index, value in enumerate(classes):
            if scores[index] > ARGS["MIN_SCORE_THRESH"]:
                if value in objects:
                    # in case it detects more that one of each object, grabs the one with higher score
                    if objects[value]['score'] > scores[index]:
                        continue
                
                point3D = PointStamped(header=Header(frame_id=ARGS["CAMERA_FRAME"]), point=Point())

                if ARGS["DEPTH_ACTIVE"] and len(self.depth_image) != 0:
                    # if frame is flipped, flip the point2D
                    point2D = get2DCentroid(boxes[index], self.depth_image)
                    #rospy.loginfo("Point2D: " + str(point2D))
                    depth = get_depth(self.depth_image, point2D) ## in m
                    #rospy.loginfo("Depth: " + str(depth))
                    #depth = depth / 1000 ## in mm
                    point3D_ = deproject_pixel_to_point(self.imageInfo, point2D, depth)
                    #rospy.loginfo("Point3D: " + str(point3D_))
                    point3D.point.x = point3D_[0]
                    point3D.point.y = point3D_[1]
                    point3D.point.z = point3D_[2]
                    pa.poses.append(Pose(position=point3D.point))

                objects[value] = {
                    "name": names[index],
                    "score": float(scores[index]),
                    "xmin": float(boxes[index][0]),
                    "ymin": float(boxes[index][1]),
                    "xmax": float(boxes[index][2]),
                    "ymax": float(boxes[index][3]),
                    "centroid_x": point2D[0],
                    "centroid_y": point2D[1],
                    "point3D": point3D
                }
        
        for label in objects:
            labelText = objects[label]["name"]
            detection = objects[label]
            res.append(objectDetection(
                    label = int(label),
                    labelText = str(labelText),
                    score = detection["score"],
                    xmin =  detection["xmin"],
                    ymin =  detection["ymin"],
                    xmax =  detection["xmax"],
                    ymax =  detection["ymax"],
                    point3D = detection["point3D"]
                ))
        # visualize here
        publish_marker_array = MarkerArray()
        
        for i, label in enumerate(objects):
            detection = objects[label]
            # generate markers for each object
            marker = Marker()
            marker.header.frame_id = ARGS["CAMERA_FRAME"]
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = detection["point3D"].point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0.5)
            publish_marker_array.markers.append(marker)
        
        # print(f"Markers: {publish_marker_array}")
        self.objects_publisher_3d.publish(publish_marker_array)
        
            # publish 
        return res
    
    def visualize_detections(self, image, boxes, classes, names, scores, colors, use_normalized_coordinates=True, max_boxes_to_draw=200, min_score_thresh=0.5, agnostic_mode=False):
        """Visualize detections on an input image."""
        
        # Convert image to BGR format (OpenCV uses BGR instead of RGB)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        for i in range(min(boxes.shape[0], max_boxes_to_draw)):
            if scores[i] > min_score_thresh:
                box = tuple(boxes[i].tolist())
                # it says it is not numeric
                color = np.array(colors[i])
                color = ( int (color [ 0 ]), int (color [ 1 ]), int (color [ 2 ])) 
                xmin, ymin, xmax, ymax = box
                if use_normalized_coordinates:
                    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
                else:
                    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
                (left, right, top, bottom) = (int(left * image.shape[1]), int(right * image.shape[1]),
                                            int(top * image.shape[0]), int(bottom * image.shape[0]))
                cv2.rectangle(image, (left, top), (right, bottom), color, 2)
                # draw label, name and score
                cv2.putText(image, f"{classes[i]}: {names[i]}: {scores[i]}", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                # draw centroid 
                cv2.circle(image, (int((left + right) / 2), int((top + bottom) / 2)), 5, (0, 0, 255), -1)
        
        # Convert image back to RGB format
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        return image

    # Main function to run the detection model.
    def run(self, frame):
        frame_processed = frame
        # frame_processed = imutils.resize(frame, width=500)

        detected_objects, visual_detections, visual_image = self.compute_result(frame_processed)

        frame = self.visualize_detections(
            visual_image,
            visual_detections['detection_boxes'],
            visual_detections['detection_classes'],
            visual_detections["detection_names"],
            visual_detections['detection_scores'],
            visual_detections['detection_colors'],
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=ARGS["MIN_SCORE_THRESH"],
            agnostic_mode=False)

        self.detections_frame = frame

        #print("PUBLISHED DATA")
        self.publisher.publish(objectDetectionArray(detections=detected_objects))
        self.fps.update()

def main():
    rospy.init_node('Vision2D', anonymous=True)
    for key in ARGS:
        ARGS[key] = rospy.get_param('~' + key, ARGS[key])
    CamaraProcessing()

if __name__ == '__main__':
    main()