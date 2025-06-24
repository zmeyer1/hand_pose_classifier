#!usr/bin/env python3
#  ros_classifier.py
#
# Use the classify_hand module within ROS Noetic
#
# Zane Meyer

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv_bridge
from classify_hand import HandClassifier
import time

def HandClassifierNode():

    def __init__(self, pickle_path = "hand_signs.pickle"):

        # Initialize the ROS node
        rospy.init_node('hand_classifier_node', anonymous=True)

        # Create a publisher to publish classification results
        self.pub = rospy.Publisher('hand_class', String, queue_size=10)
        self.sub = rospy.Subscriber('camera/image_raw', Image, self.callback)
        self.bridge = cv_bridge.CvBridge()

        # Create an instance of the HandClassifier
        self.classifier = HandClassifier(pickle_path)
        self.latest_class = None

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")

        class_name = self.classifier.classify(cv_image)

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

