#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from NGS_RC8_API import NGS_RC8_API


class API_Server(object):
    def __init__(self):
        self.api = NGS_RC8_API()

    def start_server(self):
        rospy.init_node('ngs_rc8_api_server', anonymous=True)

        rospy.Subscriber("rc8_command", String, self.issue_command_to_rc8)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def issue_command_to_rc8(self, command):
        rospy.loginfo("Issuing command to Rc8: " + command.data)
        self.api.call_function(command.data)

if __name__ == '__main__':
    try:
        print("Starting NGS RC8 api server...")
        server = API_Server()
        server.start_server()
    except rospy.ROSInterruptException:
        print("Shutting down server...")
        del server
        pass
