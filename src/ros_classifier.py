#!usr/bin/env python3
#  ros_classifier.py
#
# Use the classify_hand module within ROS Noetic
#
# Zane Meyer

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import cv2
import cv_bridge
from classify_hand import HandClassifier
import time
import message_filters

class HandClassifierNode:

    def __init__(self, pickle_path = "hand_signs.pickle"):

        # Initialize the ROS node
        rospy.init_node('hand_classifier_node', anonymous=True)

        # Create a publisher to publish classification results
        self.pub = rospy.Publisher('hand_class', String, queue_size=10)
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.callback)
        cam_sub = message_filters.Subscriber('camera/image_raw', Image)
        pc_sub = message_filters.Subscriber('camera/depth/points', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([cam_sub, pc_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        self.bridge = cv_bridge.CvBridge()

        # Create an instance of the HandClassifier
        self.classifier = HandClassifier(pickle_path)
        self.latest_class = None

    def callback(self, img_data, pc_data):
        cv_image = self.bridge.imgmsg_to_cv2(img_data, "mono8")

        class_name = self.classifier.classify(cv_image)

        if class_name != "None":
            positions = self.classifier.detector.get_positons(cv_image)
            position = positions[0] if positions else (0, 0, 0)
            # get the correct z value from the point cloud
            if pc_data:
                pc = self.bridge.pointcloud2_to_xyz_array(pc_data)
                # Assuming the point cloud is in the same coordinate system as the image
                z_value = pc[0][2] if len(pc) > 0 else 0.0
                position = (position[0], position[1], z_value)


        if class_name != self.latest_class:
            self.latest_class = class_name
            self.classified = time.time()
        elif time.time() - self.classified > 0.2:
            # If the class hasn't changed in the last 0.2 seconds, publish
            message = String()
            message.data = class_name
            self.pub.publish(message)
            self.classified = time.time() # Reset the timer after publishing

if __name__ == '__main__':
    try:
        classifier_node = HandClassifierNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Hand classifier node terminated.")

