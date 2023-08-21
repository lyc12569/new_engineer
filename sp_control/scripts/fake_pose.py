import rospy
from geometry_msgs.msg import Pose
import numpy as np

def talker():
    rospy.init_node('fake', anonymous=True)
    pub = rospy.Publisher('calibrate', Pose, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    i = 0.01
    while not rospy.is_shutdown():
        i = i+0.1
        vel_msg = Pose()
        vel_msg.position.x = 0.001+ i
        vel_msg.position.y = 0.01
        vel_msg.position.z = 0.011 +i
        vel_msg.orientation.x = 1.0+ 100 * i
        vel_msg.orientation.y = 0.0 + 100*i
        vel_msg.orientation.z = -1.0 + 100 * i
        vel_msg.orientation.w = 0.4
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass