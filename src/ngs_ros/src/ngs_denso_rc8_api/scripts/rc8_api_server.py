#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from NGS_RC8_API import NGS_RC8_API
from SocketClient import SocketClient

class API_Server(object):
    def __init__(self):
        self.api = NGS_RC8_API()

    def issue_command_to_rc8(self, command):
        rospy.loginfo("Issuing command to Rc8: ", command.data)

    def start_server(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('ngs_rc8_api_server', anonymous=True)

        rospy.Subscriber("rc8_command", String, self.issue_command_to_rc8)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    server = API_Server()
    server.start_server()
