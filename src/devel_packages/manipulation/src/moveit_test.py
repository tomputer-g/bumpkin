import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
from geometry_msgs.msg import Pose

# This script plans a straight line path from the current robot pose to pose_goal
# This plan can then be executed on the robot using the execute_plan method

# create a MoveItPlanner object and start the moveit node
franka_moveit = MoveItPlanner()

# Construct the Pose goal in panda_end_effector frame (that you read from fa.get_pose())
pose_goal = Pose()
pose_goal.position.x = 0.5843781940153249
pose_goal.position.y = 0.05791107711908864
pose_goal.position.z = 0.23098061041636195
pose_goal.orientation.x = -0.9186984147774666
pose_goal.orientation.y = 0.3942492534293267
pose_goal.orientation.z = -0.012441904611284204 
pose_goal.orientation.w = 0.020126567105018894

# convert pose goal to the panda_hand frame (the frame that moveit uses)
pose_goal_moveit = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

# plan a straight line motion to the goal
plan = franka_moveit.get_straight_plan_given_pose(pose_goal_moveit)

# execute the plan (uncomment after verifying plan on rviz)
# franka_moveit.execute_plan(plan)
