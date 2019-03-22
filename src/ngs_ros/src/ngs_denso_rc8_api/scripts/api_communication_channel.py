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
        "center" : "Move P, P( 41.63, -627.24, 734.41, -178.22, 0.95, -19.94, 5 )",
        "pos_1" : "Move P, P( 549.91, 272.17, 604.24, -179.40, -2.05, 91.77, 5 )",
        "pos_2" : "Move P, P( 295.90, 145.69, 407.22, -175.53, -1.75, 132.90, 5 )",
        "pos_3" : "Move P, P( 294.64, 141.71, 320.90, -178.42, -0.66, 132.09, 5 )",
        "pos_4" : "Move P, P( 294.10, 144.18, 314.69, -178.79, -0.54, 132.53, 5 )",
        "pos_5" : "Move P, P( 294.10, 144.16, 310.78, -179.80, -0.53, 132.53, 5 )",
        "pos_6" : "Move P, P( 293.78, 144.00, 306.16, -178.62, -0.57, 132.52, 5 )",
        "IN" : "Move P, P( 294.41, 144.46, 302.94, -177.02, -1.09, 132.41, 5 )",
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
