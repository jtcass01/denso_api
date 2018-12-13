#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from NGS_RC8_API import NGS_RC8_API
from SocketClient import SocketClient

socket_client = SocketClient()

class API_Server(object):
    def __init__(self):
        self.api = NGS_RC8_API()

    def start_server(self):
        rospy.init_node('ngs_rc8_api_server', anonymous=True)

        rospy.Subscriber("rc8_command", String, self.issue_command_to_rc8)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    @staticmethod
    def issue_command_to_rc8(command):
        rospy.loginfo("Issuing command to Rc8: ", command.data)
        socket_client.send_message(command.data + '\r')

if __name__ == '__main__':
    server = API_Server()
#    socket_client.prompt_for_connection()
    server.start_server()
