import rospy
from geometry_msgs.msg import Point
import numpy as np

def pub_target(publisher):
    arr = np.array([0, 0, 0])
    msg = Point()
    msg.x = arr[0]
    msg.y = arr[1]
    msg.z = arr[2]
    rospy.loginfo(f"{msg}")
    publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dummy_perception_node', anonymous=False)

    pub = rospy.Publisher('/target_pos', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub_target(pub)
        rate.sleep()