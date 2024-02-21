import rospy
from std_msgs.msg import String

def publish_mode():
    rospy.init_node("heartrate_publisher")  # 노드 초기화
    pub = rospy.Publisher("aircraft_status", String, queue_size=10)  # 토픽 이름과 메시지 유형 설정

    rate = rospy.Rate(1/2)  # 1초에 한 번 메시지를 publish

    while not rospy.is_shutdown():
        message = "lg_warning_on"
        pub.publish(message)  # "mode1"을 토픽에 publish
        rospy.loginfo("Published message: %s", message)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_mode()
    except rospy.ROSInterruptException:
        pass
