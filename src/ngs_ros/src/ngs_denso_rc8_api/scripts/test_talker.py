#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('rc8_command', String, queue_size=10)
    rospy.init_node('test_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rc8_command = "Draw L, V(50, 10, 50 ), Speed = 180"
        rospy.loginfo("Sending: " + rc8_command)
        pub.publish(rc8_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Initialize test talker.")
        talker()
    except rospy.ROSInterruptException:
        print("Shutting down test talker...")
        pass
