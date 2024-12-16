#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
import math
from dh_gripper_msgs.msg import GripperCtrl, GripperState, GripperRotCtrl, GripperRotState
# from tpago.src.task.scripts.ur_toycode import control_gripper


class MoveGroupInteface(object):
	def __init__(self):
		super(MoveGroupInteface, self).__init__()
		######################### setup ############################
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('ur_move_test_node', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()  # Not used in this tutorial
		group_name = "manipulator"  # group_name can be find in ur5_moveit_config/config/ur5.srdf
		self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
		
		################ Getting Basic Information ######################
		self.planning_frame = self.move_group_commander.get_planning_frame()
		print("============ Planning frame: %s" % self.planning_frame)
		self.eef_link = self.move_group_commander.get_end_effector_link()
		print("============ End effector link: %s" % self.eef_link)
		self.group_names = self.robot.get_group_names()
		print("============ Available Planning Groups:", self.robot.get_group_names())
		print("============ Printing robot state:")
		print(self.robot.get_current_state())  # get
		print("")

	def plan_cartesian_path(self, scale=1):
		waypoints = []
		wpose = self.move_group_commander.get_current_pose().pose
		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.y -= scale * 0.2  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.z -= scale * 0.15  # First move up (z)
		waypoints.append(copy.deepcopy(wpose))    

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
		(plan, fraction) = self.move_group_commander.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,      # eef_step  
										0.0)         # jump_threshold  

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		print("=========== Planning completed, Cartesian path is saved=============")
		return plan, fraction

	def execute_plan(self, plan):
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		self.move_group_commander.execute(plan, wait=True)
	
def control_gripper():
    # rospy.init_node('gripper_control_node', anonymous=True)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.loginfo("Waiting for ROS to be ready...")
    rospy.sleep(1)  # 等待ROS系统就绪
    rospy.loginfo("ROS is ready, publishing message...")

    msg = GripperCtrl()
    msg.initialize = False
    msg.position = 100.0
    msg.force = 10.0
    msg.speed = 50.0

    rospy.loginfo("GripperCtrl message: %s", msg)
    pub.publish(msg)
    rospy.loginfo("Message published to /gripper/ctrl")

    # rospy.spin()

def gripper_initialize():
    # rospy.init_node('gripper_control_node', anonymous=True)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.loginfo("Waiting for ROS to be ready...")
    rospy.sleep(1)  # 等待ROS系统就绪
    rospy.loginfo("ROS is ready, publishing message...")

    msg = GripperCtrl()
    msg.initialize = True

    rospy.loginfo("GripperCtrl message: %s", msg)
    pub.publish(msg)
    rospy.loginfo("Message published to /gripper/ctrl")
	


print("----------------------------------------------------------")
print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
print ("----------------------------------------------------------")
print("Press Ctrl-D to exit at any time")
print ("")
print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
input()
tutorial = MoveGroupInteface()
gripper_initialize()
print("============ Press `Enter` to plan and display a Cartesian path ...")
input()
cartesian_plan, fraction = tutorial.plan_cartesian_path()
print("============ Press `Enter` to execute a saved path  ...")
input()
tutorial.execute_plan(cartesian_plan)
print("============ Press `Enter` to catch ...")
input()
control_gripper()
print("============ Press `Enter` to go back ...")
input()
cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
tutorial.execute_plan(cartesian_plan)

