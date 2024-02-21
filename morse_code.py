import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received message: %s", data.data)

def listener():
    rospy.init_node('arduino_listener', anonymous=True)
    rospy.Subscriber('arduino_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()