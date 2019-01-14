#!/usr/bin/env python
# license removed for brevity
'''
'   !!!! ROS NODE !!!!
'''

import rospy
from std_msgs.msg import String

def api_communication_channel():
    '''
    '   Method Description:
    '   Author: Jacob Taylor Cassady
    '''
    print("\n===== [NGS DENSO API Communication Channel] =====")
    print("\tIssue commands 1 by 1 using PacScript conventions as described in the Denso Robotics RC8 Training Manuals")

    predefined_commands = {
        "test_draw" : "Draw L, V(50, 10, 50 ), Speed = 180",
        "test_move" : "Move P, P( 600, 50, 400, 180, 0, 180, -1)",
        "list_functions" : "Prints a list of implemented functions on the ROS Server Node"
    }


    print("\n- List of Predefined commands matching aliases -")
    for command_alias, command in predefined_commands.items():
        print("\t" + command_alias + " - " + command)

    # Creates a publisher attached to the message alias 'rc8_command'
    pub = rospy.Publisher('rc8_command', String, queue_size=10)

    # Initializes the ROS node under the alias api_communication_channel
    rospy.init_node('api_communication_channel', anonymous=True)

    # Sets the rate of the ROS node to 10hz
    rate = rospy.Rate(10)

    # Continues to loop until ROS is shutdown
    while not rospy.is_shutdown():
        # Requests a command from the user
        rc8_command = raw_input("\ndenso_rc8_api: ")

        # Checks to see if the command matches any of the predefined commands
        if rc8_command in predefined_commands.keys():
            rc8_command = predefined_commands[rc8_command]

        # Logs the command being sent
        rospy.loginfo("Sending: " + rc8_command)

        # Publishes the command across the command message
        pub.publish(rc8_command)

        # Sleeps the NODE to ensure a rate of 10hz
        rate.sleep()

if __name__ == '__main__':
    try:
        api_communication_channel()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Shutting down NGS DENSO API Communication Channel")
