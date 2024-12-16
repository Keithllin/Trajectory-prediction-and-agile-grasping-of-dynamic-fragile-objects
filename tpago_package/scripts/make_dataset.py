#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2,Image
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import cv2
import os
import open3d as o3d
import numpy as np
from datetime import datetime

bridge = CvBridge()
image_count = 0
point_cloud_count = 0
last_image = None
last_point_cloud = None
should_stop = False

def image_callback(msg):
    global last_image
    last_image = msg

def point_cloud_callback(msg):
    global last_point_cloud
    last_point_cloud = msg

def stop_timer_callback(event):
    global should_stop
    should_stop = True  # n秒后设置should_stop为True

def timer_callback(event):
    global image_count, point_cloud_count, last_image, last_point_cloud
    if should_stop:
        rospy.signal_shutdown('完成数据采集')
        return
    
    if last_image is not None:
        cv_image = bridge.imgmsg_to_cv2(last_image, "bgr8") #将相机拍摄到的图像转换为cv2格式
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")  # 获取当前时间戳
        cv2.imwrite(f"dataset/images/image_{image_count}_{timestamp}.png", cv_image) #将图像保存到dataset/images文件夹下
        image_count += 1
        last_image = None  # Reset last_image to ensure each image is processed once

    if last_point_cloud is not None:
        points_list = []
        for data in point_cloud2.read_points(last_point_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]]) #将点云数据转换为list格式
        points = np.array(points_list, dtype=np.float32) #将list格式的点云数据转换为numpy格式
        pcd = o3d.geometry.PointCloud() 
        pcd.points = o3d.utility.Vector3dVector(points) #将点云数据转换为open3d格式
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")  # 获取当前时间戳
        o3d.io.write_point_cloud(f"dataset/point_clouds/point_cloud_{point_cloud_count}_{timestamp}.pcd", pcd)
        point_cloud_count += 1
        last_point_cloud = None  # Reset last_point_cloud to ensure each point cloud is processed once

if __name__ == '__main__':
    rospy.init_node('data_capture_node')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback) #订阅相机拍摄到的图像
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback) #订阅相机拍摄到的点云数据
    os.makedirs("dataset/images", exist_ok=True)
    os.makedirs("dataset/point_clouds", exist_ok=True)
    rospy.Timer(rospy.Duration(0.5), timer_callback)  # Set timer to call timer_callback every second
    rospy.Timer(rospy.Duration(5), stop_timer_callback, oneshot=True)  # 2秒后调用stop_timer_callback一次
    rospy.spin()
    
# def image_callback(msg):
#     global image_count
#     cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#     cv2.imwrite(f"dataset/images/image_{image_count}.png", cv_image)
#     image_count += 1

# def point_cloud_callback(msg):
#     global point_cloud_count
#     points_list = []
#     for data in point_cloud2.read_points(msg, skip_nans=True):
#         points_list.append([data[0], data[1], data[2]])
#     points = np.array(points_list, dtype=np.float32)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     o3d.io.write_point_cloud(f"dataset/point_clouds/point_cloud_{point_cloud_count}.pcd", pcd)
#     point_cloud_count += 1

# if __name__ == '__main__':
#     rospy.init_node('data_capture_node')
#     rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
#     rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback)
#     os.makedirs("dataset/images", exist_ok=True)
#     os.makedirs("dataset/point_clouds", exist_ok=True)
#     rospy.spin()
    