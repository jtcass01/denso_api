#!/usr/bin/env python
# license removed for brevity
'''
'   !!!! ROS NODE !!!!
'''

import rospy
from std_msgs.msg import String

from NGS_RC8_API import NGS_RC8_API


class API_Server(object):
    '''
    '   Class Description:  An API Server packaged as a ROS Node for communicating with Denso RC8 controller
    '   Author: Jacob Taylor Cassady
    '''
    def __init__(self, socket_connection = False):
        '''
        '   Constructor
        '   Author: Jacob Taylor Cassady
        '''
        self.api = NGS_RC8_API(socket_connection = socket_connection)

    def __del__(self):
        '''
        '   Deconstructor
        '   Author: Jacob Taylor Cassady
        '''
        del self.api
        del self

    def start_server(self):
        '''
        '   Method Description: Starts the ROS node and subscribes to the rc8_command message.
        '   Author: Jacob Taylor Cassady
        '''
        rospy.init_node('ngs_rc8_api_server', anonymous=True)

        rospy.Subscriber("rc8_command", String, self.issue_command_to_rc8)

        print("\nNGS RC8 ROS API Server initialized.  Ready to relay commands to RC8.\n")

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def issue_command_to_rc8(self, command):
        '''
        '   Method Description: Logs the command and then issues it through the api.
        '   Author: Jacob Taylor Cassady
        '''
        if command.data == "list_functions":
            self.api.list_implemented_pacscript_functions()
        else:
            rospy.loginfo("Issuing command to RC8: " + command.data)
            self.api.call_function(command.data)


def run_api_server():
    try:
        print("Starting NGS Denso RC8 API server.  Below are the ROS Server options.")
        print("1 ) connect the server to an RC8 controller via TCP socket")
        print("2 ) echo the would-be messages")
        response = int(raw_input("Input server mode: "))

        if response == 1:
            print("Creating a server with a TCP connection to a Denso RC8 Robot Controller.\n")
            socket_connection = True
        elif response == 2:
            print("Creating a server in Echo mode.\n")
            socket_connection = False
        else:
            print("Server mode option passed not supported.  Defaulting to creating a TCP connection..")
            socket_connection = False

        server = API_Server(socket_connection)
        server.start_server()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Shutting down server...")
        del server
        pass

if __name__ == '__main__':
    '''
    Runs when you start up the node
    '''
    run_api_server()
