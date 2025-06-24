#!usr/bin/env python3
#
# This is a Ros Noetic service client.
#
# Zane Meyer

import rospy
from std_srvs.srv import Empty

class ServiceClient:
    def __init__(self, service_name):

        rospy.init_node('service_client_node', anonymous=True)


        rospy.wait_for_service(service_name)

        self.service_proxy = rospy.ServiceProxy(service_name, Empty)

    def call_service(self):
        try:
            # Call the service and return the response
            response = self.service_proxy()
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


if __name__ == "__main__":
    service_name = 'reset_classifier'  # Replace with your service name
    client = ServiceClient(service_name)

    # Call the service
    response = client.call_service()
    if response is not None:
        rospy.loginfo("Service called successfully.")
    else:
        rospy.logerr("Failed to call the service.")