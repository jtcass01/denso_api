#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from robot import Robot
from EthernetITV import EthernetITV

class Controller(object):
    def __init__(self, starting_state):
        self.current_state = starting_state
        self.robot_state_publisher = None

    def init_node(self):
        # Creates a publisher attached to the message alias 'robot_state'
        self.robot_state_publisher = rospy.Publisher('robot_state', String, queue_size=10)
        # Initializes the ROS node under the alias controller
        rospy.init_node('controller', anonymous=True)
        # Sets the rate of the ROS node to 1hz
        self.robot_state_rate = rospy.Rate(1) #1Hz

        print("Controller initialized with the starting state " + str(self.current_state) + ".  Please use the command prompt for directing state changes.")

        while not rospy.is_shutdown():
            # Requests a command from the user
            response = raw_input("\nLODI Controller: ")

            # Determine if command is for ethernet_ip, or robot
            if(Robot.TOOL_CHANGER_STATES.confirm_state(response)): # If command was meant for Tool Changer
                desired_state = Robot.TOOL_CHANGER_STATES.test_state(response)

                if self.current_state == desired_state:
                    rospy.loginfo("Desired and current states are the same: " + str(desired_state))
                else:
                    if desired_state is "":
                        self.current_state = Robot.TOOL_CHANGER_STATES(desired_state)
                    else:
                        self.current_state = desired_state

                    # Logs the robot_state being sent
                    rospy.loginfo("Robot state updated to: " + str(self.current_state))
                    self.robot_state_publisher.publish(self.current_state)

            elif(EthernetITV.STATES.confirm_state(response)): # If command was meant for EthernetITV
                desired_state = EthernetITV.STATES.test_state(response)
                rospy.loginfo("EthernetITV state updated to: " + str(self.current_state))
                self.robot_state_publisher.publish(desired_state)
            else:
                rospy.loginfo("Unable to resolve command " + str(response) + ". Invalid robot and/or EthernetITV state.")

            self.robot_state_rate.sleep()


    def __del__(self):
        del self.current_state
        if self.robot_state_publisher is not None:
            del self.robot_state_publisher
        if self.robot_state_rate is not None:
            del self.robot_state_rate
        del self

    @staticmethod
    def is_valid_move(current_state, desired_state):
        print("is_valid_move\n\tcurrent_state: {}\n\tdesired_state: {}".format(current_state, desired_state))
        if (current_state == Robot.TOOL_CHANGER_STATES.WAITING) or (desired_state == Robot.TOOL_CHANGER_STATES.WAITING) or (current_state == desired_state):
            return True
        elif (current_state == Robot.TOOL_CHANGER_STATES.WAITING.value) or (desired_state == Robot.TOOL_CHANGER_STATES.WAITING.value):
            return True
        else:
            return False


if __name__ == '__main__':
    starting_state = Robot.TOOL_CHANGER_STATES.get_starting_state()

    controller = Controller(starting_state = starting_state)

    try:
        controller.init_node()
    except rospy.ROSInterruptException:
        del controller
