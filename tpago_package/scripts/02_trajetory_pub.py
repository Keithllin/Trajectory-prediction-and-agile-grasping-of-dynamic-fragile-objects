#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time
import threading
import os
import math

class TrajectoryPublisher:
    def __init__(self, enable_visualization=True):
        # 初始化ROS节点
        rospy.init_node('trajectory_publisher_node', anonymous=True)
        rospy.loginfo("Trajectory Publisher Node 已启动")

        # 初始化轨迹列表和锁
        self.trajectory_3d = []
        self.trajectory_lock = threading.Lock()
        self.shutting_down = False  # 添加关闭标志

        # 可视化选项
        self.enable_visualization = enable_visualization

        # ROS订阅者和发布者
        rospy.Subscriber("/object_base_coords", Float32MultiArray, self.base_coords_callback)
        rospy.Subscriber("/save_trajectory", Bool, self.save_trajectory_callback)

        # **添加发布器**
        self.trajectory_pub = rospy.Publisher('/trajectory_3d', Float32MultiArray, queue_size=10)
        rospy.loginfo("已创建轨迹发布者 /trajectory_3d")

        if self.enable_visualization:
            self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)
            rospy.loginfo("已启用可视化")

    def base_coords_callback(self, msg):
        if self.shutting_down:
            return

        try:
            if not self.trajectory_lock.acquire(timeout=0.1):
                rospy.logdebug("无法在 base_coords_callback 中获取锁")
                return

            try:
                data = msg.data
                if not data:
                    rospy.logdebug("收到空的基座坐标数据，跳过")
                    return

                rospy.loginfo(f"处理长度为 {len(data)} 的基座坐标数据")

                # 假设每三个值为一个点的坐标 (x, y, z)
                points = [(data[i], data[i+1], data[i+2]) 
                          for i in range(0, len(data), 3)]

                if not points:
                    rospy.logwarn("从数据中未提取到有效的点")
                    return

                valid_points_count = 0
                for point in points:
                    x, y, z = point
                    if any(math.isnan(v) for v in (x, y, z)):
                        rospy.logwarn(f"跳过无效点：{point}")
                        continue

                    self.trajectory_3d.append((x, y, z))
                    valid_points_count += 1

                    # **发布当前点到 /trajectory_3d 话题**
                    trajectory_msg = Float32MultiArray()
                    trajectory_msg.data = [x, y, z]
                    self.trajectory_pub.publish(trajectory_msg)
                    rospy.loginfo(f"已发布轨迹点：{(x, y, z)}")

                rospy.loginfo(f"已向轨迹添加 {valid_points_count} 个有效点")

                if self.enable_visualization and valid_points_count > 0:
                    self.publish_trajectories()

            finally:
                self.trajectory_lock.release()

        except Exception as e:
            rospy.logerr(f"在 base_coords_callback 中发生错误：{e}")

    def publish_trajectories(self):
        try:
            marker_array = MarkerArray()

            for i, (x, y, z) in enumerate(self.trajectory_3d):
                marker = Marker()
                marker.header.frame_id = "base"  # 修改为您的基座坐标系名称
                marker.header.stamp = rospy.Time.now()
                marker.ns = "trajectory"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)

            self.marker_pub.publish(marker_array)
            rospy.logdebug(f"已发布 {len(marker_array.markers)} 个轨迹标记")

        except Exception as e:
            rospy.logerr(f"在 publish_trajectories 中发生错误：{e}")

    def save_trajectory_callback(self, msg):
        if msg.data:
            success = self.save_trajectories()
            if success:
                rospy.loginfo("已通过回调保存轨迹")
            else:
                rospy.logwarn("通过回调保存轨迹失败")

    def save_trajectories(self):
        try:
            if not self.trajectory_lock.acquire(timeout=1.0):
                rospy.logwarn("无法获取锁来保存轨迹")
                return False

            try:
                if not self.trajectory_3d:
                    rospy.loginfo("没有轨迹数据可保存")
                    return False

                np_trajectory_3d = np.array(self.trajectory_3d)

                # 使用绝对路径
                save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories")
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                    rospy.loginfo(f"已创建目录：{save_dir}")

                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"trajectory_3d_{timestamp}.npy"
                filepath = os.path.join(save_dir, filename)

                rospy.loginfo(f"正在将包含 {len(self.trajectory_3d)} 个点的轨迹保存到 {filepath}")
                np.save(filepath, np_trajectory_3d)
                rospy.loginfo(f"成功将轨迹保存到 {filepath}")
                return True

            finally:
                self.trajectory_lock.release()

        except Exception as e:
            rospy.logerr(f"在 save_trajectories 中发生错误：{e}")
            return False

    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(f"在 run 方法中发生错误：{e}")
        finally:
            self.shutting_down = True
            if self.save_trajectories():
                rospy.loginfo("最终轨迹保存成功")
            else:
                rospy.logwarn("最终轨迹保存失败")

if __name__ == '__main__':
    try:
        traj_publisher = TrajectoryPublisher()
        traj_publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 中断收到")
    except Exception as e:
        rospy.logerr(f"发生未预料的错误：{e}")