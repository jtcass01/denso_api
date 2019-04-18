#!/usr/bin/env python
import enum
import rospy
from std_msgs.msg import String
from FileManager import FileManager
from EthernetITV import EthernetITV

class Robot(object):
    def __init__(self, starting_state, ethernet_ip_ip_address):
        self.current_state = starting_state
        self.command_publisher =  None
        self.command_rate = None
        self.ethernet_ip = EthernetITV(ip_address = ethernet_ip_ip_address)

    def __del__(self):
        del self.current_state
        if self.command_publisher is not None:
            del self.command_publisher
        if self.command_rate is not None:
            del self.command_rate
        del self

    def init_node(self):
        # Creates a publisher attached to the message alias 'rc8_command'
        self.command_publisher = rospy.Publisher('rc8_command', String, queue_size=10)

        # Initializes the ROS node under the alias api_communication_channel
        rospy.init_node('robot', anonymous=True)

        # Sets the rate of the ROS node to 1hz
        self.command_rate = rospy.Rate(0.2)

        # Creates a subscriber attached to the message alias 'robot_state'
        rospy.Subscriber('robot_state', String, self.change_states)

        print("Robot initialized with the starting state " + str(self.current_state) + ".  Waiting for command sets...")

        rospy.spin()

    def change_states(self, subscriber_data):
        command = subscriber_data.data

        # Determine if command is for ethernet_ip, or robot
        if(Robot.TOOL_CHANGER_STATES.confirm_state(command)): # If command was meant for Tool Changer
            desired_state = Robot.TOOL_CHANGER_STATES(Robot.TOOL_CHANGER_STATES.test_state(command))
            rospy.loginfo("Robot recieved command to move to " + str(desired_state))

            if self.is_valid_state(self.current_state, desired_state):
                if desired_state == Robot.TOOL_CHANGER_STATES.WAITING:
                    self.come_from(self.current_state)
                    self.current_state = Robot.TOOL_CHANGER_STATES.WAITING
                elif self.current_state == Robot.TOOL_CHANGER_STATES.WAITING:
                    self.goto(desired_state)
                    self.current_state = desired_state
            else:
                if self.is_valid_state(self.current_state, Robot.TOOL_CHANGER_STATES.WAITING) and self.is_valid_state(Robot.TOOL_CHANGER_STATES.WAITING, desired_state):
                    self.come_from(self.current_state)
                    self.current_state = Robot.TOOL_CHANGER_STATES.WAITING
                    self.goto(desired_state)
                    self.current_state = desired_state
                else:
                    rospy.loginfo("Robot deemed this was not a valid move. \n\tcurrent_state: " + str(self.current_state) + "\n\tdesired_state: " + str(response))

        elif(EthernetITV.STATES.confirm_state(command)): # If command was meant for EthernetITV
            desired_state = EthernetITV.STATES(EthernetITV.STATES.test_state(command))
            rospy.loginfo("Robot recieved command to update EthernetITV to: " + str(desired_state))

            if EthernetITV.STATES.Pressure_ON == desired_state:
                self.ethernet_ip.turn_on()
            elif EthernetITV.STATES.Pressure_OFF == desired_state:
                self.ethernet_ip.turn_off()

            rospy.loginfo("EthernetITV state updated to: " + str(desired_state))
        else:
            rospy.loginfo("Unable to resolve command " + str(command) + ". Invalid robot and/or EthernetITV state.")


    def goto(self, state):
        rospy.loginfo("BEGIN goto " + state.value)

        command_set = FileManager.load_command_set(state.value)

        rospy.loginfo("command_set loaded: " + " ".join("\n" + str(command_index) + ": " + str(command) for command_index, command in enumerate(command_set)))

        for command in command_set:
            rospy.loginfo("Command issued: " + str(command))
            self.command_publisher.publish(str(command))
            self.command_rate.sleep()

        rospy.loginfo("END goto " + state.value)

    def come_from(self, state):
        rospy.loginfo("BEGIN come_from " + state.value)

        if isinstance(self.current_state, Robot.TOOL_CHANGER_STATES):
            command_set = FileManager.load_command_set(self.current_state.value)[::-1]
        else:
            command_set = FileManager.load_command_set(self.current_state)[::-1]

        rospy.loginfo("command_set loaded: " + " ".join("\n" + str(command_index) + ": " + str(command) for command_index, command in enumerate(command_set)))

        for command in command_set:
            rospy.loginfo("Command issued: " + str(command))
            self.command_publisher.publish(str(command))
            self.command_rate.sleep()

        rospy.loginfo("END come_from " + state.value)

    @staticmethod
    def is_valid_state(current_state, desired_state):
        if current_state == Robot.TOOL_CHANGER_STATES.WAITING or desired_state == Robot.TOOL_CHANGER_STATES.WAITING or current_state == desired_state:
            return True
        else:
            return False

    class TOOL_CHANGER_STATES(enum.Enum):
        SMALL_ELECTRIC_GRIPPER = 'small_electric_gripper'
        MEDIUM_ELECTRIC_GRIPPER = 'medium_electric_gripper'
        LARGE_ELECTRIC_GRIPPER = 'large_electric_gripper'
        WAITING = 'waiting'

        @staticmethod
        def test_state(suggested_state):
            switcher = {
                Robot.TOOL_CHANGER_STATES.SMALL_ELECTRIC_GRIPPER.value : 'small_electric_gripper',
                Robot.TOOL_CHANGER_STATES.MEDIUM_ELECTRIC_GRIPPER.value : 'medium_electric_gripper',
                Robot.TOOL_CHANGER_STATES.LARGE_ELECTRIC_GRIPPER.value : 'large_electric_gripper',
                Robot.TOOL_CHANGER_STATES.WAITING.value : 'waiting'
            }

            return switcher.get(suggested_state, "invalid_robot_state")

        @staticmethod
        def confirm_state(suggested_state):
            suggested_state = Robot.TOOL_CHANGER_STATES.test_state(suggested_state)

            if suggested_state != "invalid_robot_state":
                return True
            else:
                return False

        @staticmethod
        def get_starting_state():
            user_state_response = raw_input("\nWhat is the current state of the robot: ")

            if(Robot.TOOL_CHANGER_STATES.confirm_state(user_state_response)): # If command was meant for Tool Changer
                desired_state = Robot.TOOL_CHANGER_STATES.test_state(user_state_response)
                starting_state = Robot.TOOL_CHANGER_STATES(desired_state)
            else:
                starting_state = Robot.TOOL_CHANGER_STATES.WAITING

            return starting_state

def get_ethernet_itv_ip_address():
    ip_address = raw_input("\nWhat is ip address of the robot's ethernet ITV [default = 192.168.1.20]: ")

    if ip_address == "":
        ip_address = "192.168.1.20"

    return ip_address

if __name__ == "__main__":
    starting_state = Robot.TOOL_CHANGER_STATES.get_starting_state()
    ethernet_itv_ip_address = get_ethernet_itv_ip_address()
    robot = Robot(starting_state = starting_state, ethernet_ip_ip_address = ethernet_itv_ip_address)

    try:
        robot.init_node()
    except rospy.ROSInterruptException:
        del robot
