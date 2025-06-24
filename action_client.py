#!usr/bin/env python3
#
# This is a Ros Noetic action client.
#
# Zane Meyer

import rospy 
import actionlib


class ActionClient:
    def __init__(self, action_name):
        self.client = actionlib.SimpleActionClient(action_name, actionlib.msgs.GoalID)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal)

    def wait_for_result(self):
        return self.client.wait_for_result()

    def get_result(self):
        return self.client.get_result()

    def cancel_goal(self):
        self.client.cancel_goal()

if __name__ == '__main__':
    rospy.init_node('action_client_node')
    action_name = rospy.get_param('~action_name', 'example_action')
    
    client = ActionClient(action_name)
    
    # Example goal, replace with actual goal type
    goal = actionlib.msgs.GoalID()
    
    client.send_goal(goal)
    
    if client.wait_for_result():
        result = client.get_result()
        rospy.loginfo("Action completed successfully with result: %s", result)
    else:
        rospy.logwarn("Action did not complete in time.")