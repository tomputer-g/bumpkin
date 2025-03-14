import rospy
from geometry_msgs.msg import Point


def motion_tracker(msg):
    # convert target ee point to joint positions 
    # plan and execute path to target 
    pass

if __name__ == '__main__':
    rospy.init_node('motion_tracking_node')

    rospy.Subscriber('/target_pos', Point, motion_tracker)

    rospy.spin()