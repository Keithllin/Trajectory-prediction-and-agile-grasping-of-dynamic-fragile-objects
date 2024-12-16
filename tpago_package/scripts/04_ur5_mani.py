#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, PointStamped
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from dh_gripper_msgs.msg import GripperCtrl
import tf2_ros
import tf2_geometry_msgs

class MoveGroupInterface(object):
    def __init__(self):
        super(MoveGroupInterface, self).__init__()
        ######################### 初始化 ############################
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur_grasp_trajectory_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"  # 机械臂的规划组名称
        self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        ################ 获取基本信息 ######################
        self.planning_frame = self.move_group_commander.get_planning_frame()
        print("============ 规划参考坐标系: %s" % self.planning_frame)
        self.eef_link = self.move_group_commander.get_end_effector_link()
        print("============ 末端执行器链接: %s" % self.eef_link)
        self.group_names = self.robot.get_group_names()
        print("============ 可用的规划组:", self.robot.get_group_names())
        print("============ 机器人状态:")
        print(self.robot.get_current_state())
        print("")

        ########### 初始化 tf2 监听器 ###########
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def plan_to_pose_goal(self, target_pose):
        """
        规划到给定的位姿目标。

        :param target_pose: Pose 对象，定义目标的位置和姿态。
        :return: 如果规划和执行成功，返回 True，否则返回 False。
        """
        self.move_group_commander.set_pose_target(target_pose)
        success = self.move_group_commander.go(wait=True)
        self.move_group_commander.stop()
        self.move_group_commander.clear_pose_targets()
        return success

    def control_gripper(self, position, force=50.0, speed=100.0):
        pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
        rospy.loginfo("Waiting for ROS to be ready...")
        rospy.sleep(1)  # 等待ROS系统就绪
        rospy.loginfo("ROS is ready, publishing message...")
        msg = GripperCtrl()
        msg.initialize = False
        msg.position = position
        msg.force = force
        msg.speed = speed
        pub.publish(msg)
        

def main():
    try:
        tutorial = MoveGroupInterface()

        # 初始化机械爪
    

        # 移动到初始位置
        # tutorial.move_group_commander.set_named_target("initialization")
        # success = tutorial.move_group_commander.go()
        # if success:
        #     rospy.loginfo("成功移动到初始位置")
        # else:
        #     rospy.logwarn("移动到初始位置失败")
        pose_target = tutorial.move_group_commander.get_current_pose().pose
        rospy.loginfo(f"============ 当前位姿: {pose_target}")

        # 定义预测位置的回调函数
        def predicted_position_callback(msg):
            data = msg.data  # 预期为 [x, y, z]，基座坐标系下
            if len(data) < 3:
                rospy.logwarn("接收到不完整的预测位置信息")
                return
            x_base, y_base, z_base = data[:3]

            rospy.loginfo(f"接收到预测位置 (基座坐标系): x={x_base}, y={y_base}, z={z_base}")

            # 准备目标位姿
            target_pose = Pose()
            target_pose.position.x = - x_base / 1000
            target_pose.position.y = y_base / 1000
            target_pose.position.z = z_base / 1000 -0.035
            target_pose.orientation = pose_target.orientation  # 保持当前姿态
            rospy.loginfo(f"目标位姿: Position=({x_base}, {y_base}, {z_base}), Orientation=({target_pose.orientation.x}, {target_pose.orientation.y}, {target_pose.orientation.z}, {target_pose.orientation.w})")

            # 规划并执行到目标位姿
            success = tutorial.plan_to_pose_goal(target_pose)

            if success:
                rospy.loginfo("规划并执行到目标位姿成功")
                # 闭合机械爪进行抓取
                tutorial.control_gripper(position=100.0)
                rospy.loginfo("抓取成功")
            else:
                rospy.logwarn("规划或执行到目标位姿失败")

            # 返回初始位置
            # tutorial.move_group_commander.set_named_target("initialization")
            # success = tutorial.move_group_commander.go()
            print("============ Press `Enter` to go back ...")
            input()
            success = tutorial.plan_to_pose_goal(pose_target)
            if success:
                rospy.loginfo("成功返回初始位置")
            else:
                rospy.logwarn("返回初始位置失败")
            # 打开机械爪
            # tutorial.control_gripper(position=1000.0)
            # rospy.loginfo("机械爪已打开")

        # 订阅预测位置的话题
        rospy.Subscriber('/predicted_positions', Float32MultiArray, predicted_position_callback)
        rospy.loginfo("已订阅 /predicted_positions 话题")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生未预料的错误：{e}")

if __name__ == '__main__':
    main()