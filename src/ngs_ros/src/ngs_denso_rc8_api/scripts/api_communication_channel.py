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
        "test_draw" : "Draw L, V( 30, 10, 30 )",
        "test_move" : "Move P, P( 600, 50, 400, 180, 0, 180, -1 )",
        "move_1" : "Move P, P( -150.69, -605.59, 884.18, -178.59, -0.90, -18.70, 5 )",
        "move_2" : "Move P, P( -284.69, -557.72, 867.11, -179, -0.47, 5.64, 5 )",
        "test_drive" : "Drive (1,10), (7,70), (8, 18)",
        "list_functions" : "list_functions"                                     # Prints a list of implemented functions on the ROS Server Node
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
