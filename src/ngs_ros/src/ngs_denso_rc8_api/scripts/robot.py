#!/usr/bin/env python
import enum
import rospy
from std_msgs.msg import String
from FileManager import FileManager
from EthernetITV import EthernetITV
from requests.exceptions import ConnectionError

class Robot(object):
    def __init__(self, tool, ethernet_itv_ip_address, ethernet_itv_pressure_target):
        self.tool = tool
        self.command_publisher =  None
        self.command_rate = None
        self.ethernet_itv = EthernetITV(ip_address = ethernet_itv_ip_address, pressure_target = ethernet_itv_pressure_target)

    def __del__(self):
        del self.tool
        del self.ethernet_itv
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
        rospy.Subscriber('robot_state', String, self.recieve_command)

        print("Robot initialized with the tool " + str(self.tool.name) + ".  Waiting for command sets...")

        rospy.spin()

    def recieve_command(self, command_data):
        command = command_data.data

        if FileManager.is_valid_tool_path(command) or command == "None":
            if command == self.tool or (self.tool is None and command == "None"):
                rospy.loginfo("Command recieved for robot to use tool: " + command + ", but tool changer already thinks it is using that tool.")
                return

            if command == "None":                                  # Return the current tool
                self.goto(self.tool.name)                          # Goto tool location
                self.deactivate_itv()                              # Drop the current tool.
                self.come_from(self.tool.name)                     # Return from current tool
                self.tool.pneumatic_status = False                 # Update tool attributes
                self.tool.name = None
            else:
                self.grab_tool(command)
        else:
            rospy.loginfo("Unable to resolve robot command " + str(command) + ".")

    def grab_tool(self, new_tool_name):
        if self.tool.name is None:                         # Not currently holding a tool, go get one.
            self.goto(new_tool_name)                       # Go to new tool.
            self.activate_itv()                            # Activate pressure to grip tool
            self.come_from(new_tool_name)                  # Return to center with new tool.
            self.tool.name = new_tool_name                 # Update tool attributes to match new tool.
            if "pneumatic" in new_tool_name:
                self.tool.pneumatic_status = True
            else:
                self.tool.pneumatic_status = False
        else:
            old_tool = self.tool.name
            self.goto(old_tool)                            # Return the current tool where it needs to go.
            self.deactivate_itv()                          # Drop the current tool.
            self.come_from(old_tool)                       # Return from the old tools location.
            self.tool.name = None                          # Update tool attributes to match lack of tool
            self.tool.pneumatic_status = False
            self.goto(new_tool_name)                       # Go to new tool.
            self.activate_itv()                            # Activate pressure to grip tool
            self.come_from(new_tool_name)                  # Return to center with new tool.
            self.tool.name = new_tool_name                 # Update tool attributes to match new tool.
            if "pneumatic" in new_tool_name:
                self.tool.pneumatic_status = True
            else:
                self.tool.pneumatic_status = False

    def goto(self, tool_name):
        rospy.loginfo("BEGIN goto " + str(tool_name))

        path = FileManager.load_path(tool_name)

        rospy.loginfo("Path loaded: " + " ".join("\n" + str(command_index) + ": " + str(command) for command_index, command in enumerate(path)))

        for command in path:
            rospy.loginfo("Command issued: " + str(command))
            self.command_publisher.publish(str(command))
            self.command_rate.sleep()

        rospy.loginfo("END goto " + tool_name)

    def come_from(self, tool_name):
        rospy.loginfo("BEGIN come_from " + tool_name)

        path = FileManager.load_path(tool_name)[::-1]

        rospy.loginfo("Path loaded: " + " ".join("\n" + str(command_index) + ": " + str(command) for command_index, command in enumerate(path)))

        for command in path:
            rospy.loginfo("Command issued: " + str(command))
            self.command_publisher.publish(str(command))
            self.command_rate.sleep()

        rospy.loginfo("END come_from " + tool_name)

    def activate_itv(self):
        try:
            self.ethernet_itv.turn_on()                    # Activate pressure to grip tool
            rospy.loginfo("Ethernet ITV's pressure set to pressure target")
        except ConnectionError:
            rospy.loginfo("Error connecting to ethernet_itv during activation.")
        rospy.sleep(2.)

    def deactivate_itv(self):
        try:
            self.ethernet_itv.turn_off()                       # Drop the current tool
            rospy.loginfo("Ethernet ITV's pressure set to 0")
        except ConnectionError:
            rospy.loginfo("Error connecting to ethernet_itv during deactivation.")
        rospy.sleep(2.)

    @staticmethod
    def get_ethernet_itv_info():
        ip_address = Robot.get_ethernet_itv_ip_address()
        pressure_target = Robot.get_ethernet_itv_pressure_target()

        return ip_address, pressure_target

    @staticmethod
    def get_ethernet_itv_pressure_target():
        pressure_target = int(raw_input("Pressure target for the Ethernet ITV: "))
        return pressure_target

    @staticmethod
    def get_ethernet_itv_ip_address():
        ip_address = raw_input("\nWhat is ip address of the robot's ethernet ITV [default = 192.168.1.20]: ")

        if ip_address == "":
            ip_address = "192.168.1.20"

        return ip_address

    @staticmethod
    def perform_tool_check():
        tool_check = ""
        tool = Robot.Tool(None, False)

        while tool_check != "y" and tool_check != "n":
            tool_check = raw_input("\nIs the robot currently gripping a tool (y/n): ")

            if tool_check == "y":
                FileManager.list_tool_paths()
                tool_name = raw_input("\nWhich of the above listed tool paths does the tool match (if none, please cancel program and update tool_paths directory for your tool):")

                if FileManager.is_valid_tool_path(tool_name):
                    tool.name = tool_name
                    if "pneumatic" in tool_name:
                        tool.pneumatic_status = True
                else:
                    print("The above tool does not match any paths found in the tool_paths directory.  Please cancel the program, update the directory and try again.")

            elif tool_check == "n":
                tool.pneumatic_status = False

        return tool

    @staticmethod
    def get_initial_info():
        tool = Robot.perform_tool_check()
        ethernet_itv_ip_address, ethernet_itv_pressure_target = Robot.get_ethernet_itv_info()

        return tool, ethernet_itv_ip_address, ethernet_itv_pressure_target

    class Tool(object):
        def __init__(self, name, pneumatic_status):
            self.name = name
            self.pneumatic_status = pneumatic_status

        def __del__(self):
            if self.name is not None:
                del self.name
            del self.pneumatic_status
            del self

if __name__ == "__main__":
    tool, ethernet_itv_ip_address, ethernet_itv_pressure_target = Robot.get_initial_info()
    robot = Robot(tool, ethernet_itv_ip_address, ethernet_itv_pressure_target)

    try:
        robot.init_node()
    except rospy.ROSInterruptException:
        del robot
