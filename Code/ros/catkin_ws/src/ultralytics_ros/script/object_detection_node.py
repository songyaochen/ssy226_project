#!/usr/bin/env python3

import cv_bridge
import roslib.packages
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from detection_msgs.msg import DetectionDepth, DetectionDepthArray


class TrackerNode:
    def __init__(self):
        yolo_model = rospy.get_param("~yolo_model", "yolov8n.pt")
        detection_topic = rospy.get_param("~detection_topic", "detection_result")
        image_topic = rospy.get_param("~image_topic", "image_raw")
        depth_topic = rospy.get_param("~depth_topic", "depth_raw")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.max_det = rospy.get_param("~max_det", 300)
        self.classes = rospy.get_param("~classes", None)
        self.tracker = rospy.get_param("~tracker", "bytetrack.yaml")
        self.debug = rospy.get_param("~debug", False)
        self.debug_conf = rospy.get_param("~debug_conf", True)
        self.debug_line_width = rospy.get_param("~debug_line_width", None)
        self.debug_font_size = rospy.get_param("~debug_font_size", None)
        self.debug_font = rospy.get_param("~debug_font", "Arial.ttf")
        self.debug_labels = rospy.get_param("~debug_labels", True)
        self.debug_boxes = rospy.get_param("~debug_boxes", True)
        path = roslib.packages.get_pkg_dir("ultralytics_ros")
        self.model = YOLO(f"{path}/models/{yolo_model}")
        self.joy_priority = False

        self.depth_image = None
        self.sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )
        self.sub_depth = rospy.Subscriber(
            depth_topic, Image, self.save_depth_image, queue_size=1, buff_size=2**24
        )
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.detection_pub = rospy.Publisher(
            detection_topic, DetectionDepthArray, queue_size=1
        )
        self.bridge = cv_bridge.CvBridge()


    def image_callback(self, msg):
        header = msg.header
        numpy_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.track(
            source=numpy_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            classes=self.classes,
            tracker=self.tracker,
            verbose=False
        )
        depth_image = self.depth_image
        self.publish_detection(results, header, depth_image)
        self.publish_debug_image(results)

    def save_depth_image(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def publish_debug_image(self, results):
        if self.debug and results is not None:
            plotted_image = results[0].plot(
                conf=self.debug_conf,
                line_width=self.debug_line_width,
                font_size=self.debug_font_size,
                font=self.debug_font,
                labels=self.debug_labels,
                boxes=self.debug_boxes,
            )
            debug_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
            self.image_pub.publish(debug_image_msg)

    def publish_detection(self, results, header, depth_image):
        if results is not None and results[0].masks is not None:
            detections_msg = DetectionDepthArray()
            detections_msg.header = header
            bounding_box = results[0].boxes.xywh
            masks = results[0].masks.data.numpy().copy()
            classes = results[0].boxes.cls
            confidence_score = results[0].boxes.conf
            for bbox, mask, cls, conf in zip(bounding_box, masks, classes, confidence_score):
                mask[mask==0]=['nan']
                obj_depth = np.multiply(mask, depth_image)
                depth = np.nanmean(obj_depth)
                detection = DetectionDepth()
                detection.bbox.center.x = float(bbox[0])
                detection.bbox.center.y = float(bbox[1])
                detection.bbox.size_x = float(bbox[2])
                detection.bbox.size_y = float(bbox[3])
                detection.depth = depth
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(cls)
                hypothesis.score = float(conf)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)
            self.detection_pub.publish(detections_msg)


if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    rospy.spin()
