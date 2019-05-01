#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from robot import Robot
from EthernetITV import EthernetITV
from FileManager import FileManager

class Controller(object):
    def __init__(self, tool):
        self.tool = tool
        self.robot_state_publisher = None

    def init_node(self):
        # Creates a publisher attached to the message alias 'robot_state'
        self.robot_state_publisher = rospy.Publisher('robot_state', String, queue_size=10)
        # Initializes the ROS node under the alias controller
        rospy.init_node('controller', anonymous=True)
        # Sets the rate of the ROS node to 1hz
        self.robot_state_rate = rospy.Rate(1) #1Hz

        print("Controller initialized with the starting tool " + str(self.tool.name) + ".  Please use the command prompt for directing tool changes.")
        FileManager.list_tool_paths()

        while not rospy.is_shutdown():
            # Requests a command from the user
            response = raw_input("\nLODI Tool Changer Controller: ")

            # Determine if response is valid
            if FileManager.is_valid_tool_path(response) or response == "None":
                rospy.loginfo("Sending command to tool path to tool changer: " + str(response))
                self.robot_state_publisher.publish(response)
            else:
                print("TO RETURN TOOLS USE PATH 'None'")
                rospy.loginfo("The tool " + str(response) + " does not match any paths found in the tool_paths directory.")

            self.robot_state_rate.sleep()

    def __del__(self):
        del self.tool
        if self.robot_state_publisher is not None:
            del self.robot_state_publisher
        if self.robot_state_rate is not None:
            del self.robot_state_rate
        del self

if __name__ == '__main__':
    tool = Robot.perform_tool_check()
    controller = Controller(tool = tool)

    try:
        controller.init_node()
    except rospy.ROSInterruptException:
        del controller
