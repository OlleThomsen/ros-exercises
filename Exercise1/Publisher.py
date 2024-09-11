import rospy
from std_msgs.msg import Int32

def publisher():
    pub = rospy.Publisher('/Thomsen', Int32, queue_size=10)
    rospy.init_node('nodeA', anonymous=True)
    rate = rospy.Rate(20)  # 20 Hz
    k = 0
    n = 4

    while not rospy.is_shutdown():
        k += n
        rospy.loginfo(k)
        pub.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass