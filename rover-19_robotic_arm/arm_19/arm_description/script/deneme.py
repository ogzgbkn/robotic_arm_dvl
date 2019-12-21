#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64 as F64


def talker():
    pub1 = rospy.Publisher('/rover_arm_eksen1_joint_position_controller/command', F64, queue_size=1)
    pub2 = rospy.Publisher('/rover_arm_eksen2_joint_position_controller/command', F64, queue_size=1)
    pub3 = rospy.Publisher('/rover_arm_eksen3_joint_position_controller/command', F64, queue_size=1)
    pub4 = rospy.Publisher('/rover_arm_eksen4_joint_position_controller/command', F64, queue_size=1)
    pub5 = rospy.Publisher('/rover_arm_eksen5_joint_position_controller/command', F64, queue_size=1)
    pub6 = rospy.Publisher('/rover_arm_eksen6_joint_position_controller/command', F64, queue_size=1)
    rospy.init_node('control_Deneme', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pubme = []
    for i in range(1,7):
        a = F64()
        a.data = i/6.0
        pubme.append(a)
    while not rospy.is_shutdown():
        pub1.publish(pubme[0])
        pub2.publish(pubme[1])
        pub3.publish(pubme[2])
        pub4.publish(pubme[3])
        pub5.publish(pubme[4])
        pub6.publish(pubme[5])

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass