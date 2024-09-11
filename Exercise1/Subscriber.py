import rospy
from std_msgs.msg import Int32

def callback(data):
    q = 0.5
    result = data.data / q
    rospy.loginfo(result)
    pub.publish(result)

def subscriber():
    rospy.init_node('nodeB', anonymous=True)
    rospy.Subscriber('/Thomsen', Int32, callback)
    global pub
    pub = rospy.Publisher('/kthfs/result', Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass