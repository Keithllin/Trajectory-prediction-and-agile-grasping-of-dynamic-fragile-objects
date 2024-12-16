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
		self.scene = moveit_commander.PlanningSceneInterface()  
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
	
	def plan_to_pose_goal(self, target_pose):
		"""
		Plans to a given pose goal in the Cartesian space.
	
		:param target_pose: A Pose object defining the goal position and orientation.
		:return: A tuple (plan, success) where plan is the computed trajectory and success is a boolean indicating if planning was successful.
		"""
		self.move_group_commander.set_pose_target(target_pose)
		# Now, we call the planner to compute the plan and execute it.
		# plan = self.move_group_commander.plan()
	
		# Check if planning was successful
		success = self.move_group_commander.go(wait=True)

		self.move_group_commander.stop()

		self.move_group_commander.clear_pose_targets()
	
		# if success:
		# 	print("============ Planning to pose goal succeeded ============")
		# else:
		# 	print("============ Planning to pose goal failed ============")
	
		return success

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
	

def control_gripper():
    # rospy.init_node('gripper_control_node', anonymous=True)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.loginfo("Waiting for ROS to be ready...")
    rospy.sleep(1)  # 等待ROS系统就绪
    rospy.loginfo("ROS is ready, publishing message...")

    msg = GripperCtrl()
    msg.initialize = False
    msg.position = 50.0
    msg.force = 10.0
    msg.speed = 90.0

    rospy.loginfo("GripperCtrl message: %s", msg)
    pub.publish(msg)
    rospy.loginfo("Message published to /gripper/ctrl")

    # rospy.spin()


print("Press Ctrl-D to exit at any time")
print ("")
print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
input()
tutorial = MoveGroupInteface()
gripper_initialize()
# tutorial.move_group_commander.set_named_target("initialization")
# plan = tutorial.move_group_commander.go()
pose_target = tutorial.move_group_commander.get_current_pose().pose
print("============ Current pose: ", pose_target)
# print("============ Press `Enter` to plan and display a Cartesian path ...")
# input()
# cartesian_plan, fraction = tutorial.plan_cartesian_path()
# print("============ Press `Enter` to execute a saved path  ...")
# input()
# tutorial.execute_plan(cartesian_plan)
print("============ Press `Enter` to plan to a specific pose goal ...")
input()

# # 定义目标位置
# target_pose = Pose()
# target_pose.orientation.w = 1.0
# target_pose.position.x = 0.4  # 将 x, y, z 替换为你目标位置的坐标
# target_pose.position.y = 0.2
# target_pose.position.z = 0.4
# # 将 TCP 坐标转换为目标位置
# tcp_x = -0.1071  # in meters
# tcp_y = -0.4875  # in meters
# tcp_z = 0.4304   # in meters

# # 目标点在TCP正前方，并向下偏移100mm（0.1m）
target_x = -0.12785827148925429
target_y = 0.5702951205832137
target_z = 0.4511383113638684

target_pose = geometry_msgs.msg.Pose()
# target_pose.orientation.w = 1  # 假设没有旋转
target_pose.position.x = target_x
target_pose.position.y = target_y
target_pose.position.z = target_z
target_pose.orientation = pose_target.orientation  # 保持当前姿态


# 使用新的规划函数
success = tutorial.plan_to_pose_goal(target_pose)
if success==True:
    # print("============ Press `Enter` to initialize the gripper to the target pose ...")
    # input()
    # gripper_initialize()
    
    print("============ Press `Enter` to catch ...")
    input()
    control_gripper()
else:
    print("Planning failed. Please try a different pose or check the configuration.")
# print("============ Press `Enter` to catch ...")
# input()
# control_gripper()
print("============ Press `Enter` to go back ...")
input()
# cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
# tutorial.execute_plan(cartesian_plan)
success = tutorial.plan_to_pose_goal(pose_target)
# tutorial.move_group_commander.set_named_target(pose_target)
# plan = tutorial.move_group_commander.go()