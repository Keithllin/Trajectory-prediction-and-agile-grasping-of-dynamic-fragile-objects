#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
import cv2
import numpy as np
from sensor_msgs import point_cloud2
import pyrealsense2 as rs
import threading
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

class YOLODetectionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('yolo_detection_node', anonymous=True)
        rospy.loginfo("YOLO detection node started")

        # 初始化YOLO模型
        model_path = "/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/tpago/runs/detect/train4/weights/best.pt"
        self.model = YOLO(model_path)
        rospy.loginfo(f"Loaded YOLO model from {model_path}")

        # 发布检测到的物体的边框和深度信息
        self.bbox_publisher = rospy.Publisher("/object_bbox", Float32MultiArray, queue_size=10)
        self.base_coords_publisher = rospy.Publisher("/object_base_coords", Float32MultiArray, queue_size=10)
        rospy.loginfo("Publishers initialized")

        # 定义检测到的物体深度范围
        self.min_depth = 0.1
        self.max_depth = 1.5

        # 占位符，用于存储最新的点云数据和相机信息
        self.latest_point_cloud = None
        self.latest_camera_info = None
        self.latest_depth_image = None
        self.latest_color_image = None  # 存储最新的彩色图像

        # 存储最新的中点信息
        self.midpoints = []
        self.midpoints_lock = threading.Lock()

        # 订阅相关话题
        rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        rospy.loginfo("Subscribers initialized")

        # 定时器用于每隔1秒输出中点信息
        rospy.Timer(rospy.Duration(1), self.print_midpoints)

        # 销毁OpenCV窗口的回调
        rospy.on_shutdown(self.shutdown_callback)

        # 初始化CvBridge
        self.bridge = CvBridge()

        # 手眼标定的转换矩阵，从 /camera_color_frame 到 /base
        # 平移部分
        self.translation = np.array([-0.152, -1.030, 0.175])

        # 旋转部分，四元数 (x, y, z, w)
        self.quaternion = np.array([-0.631, 0.023, -0.016, 0.776])

        # 构建转换矩阵 T_base_camera
        self.T_base_camera = self.get_transformation_matrix(self.translation, self.quaternion)
        rospy.loginfo("Transformation matrix from camera to base computed")

    def get_transformation_matrix(self, translation, quaternion):
        # 将四元数转换为旋转矩阵
        rotation_matrix = quaternion_matrix(quaternion)
        # 构建转换矩阵
        transformation_matrix = rotation_matrix
        transformation_matrix[0:3, 3] = translation
        return transformation_matrix

    def transform_camera_to_base(self, x_cam, y_cam, z_cam):
        # 构建物体在相机坐标系下的齐次坐标
        point_cam = np.array([x_cam, y_cam, z_cam, 1]).reshape(4, 1)
        # 使用转换矩阵计算物体在基座坐标系下的坐标
        point_base = np.dot(self.T_base_camera, point_cam)
        x_base, y_base, z_base = point_base[0:3, 0]
        return x_base, y_base, z_base

    def pointcloud_callback(self, msg):
        self.latest_point_cloud = msg
        rospy.logdebug("Received new point cloud data")

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg
        rospy.logdebug("Received new camera info")

    def depth_image_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            rospy.logdebug("Received new depth image")
        except Exception as e:
            rospy.logwarn(f"Failed to convert depth image: {e}")

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth, camera_info):
        intrinsics = rs.intrinsics()
        intrinsics.width = camera_info.width
        intrinsics.height = camera_info.height
        intrinsics.ppx = camera_info.K[2]
        intrinsics.ppy = camera_info.K[5]
        intrinsics.fx = camera_info.K[0]
        intrinsics.fy = camera_info.K[4]
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = camera_info.D

        # 将像素坐标和深度转换为相机坐标系下的物体坐标
        result = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)

        x_cam = result[0]  # x in meters
        y_cam = result[1]
        z_cam = result[2]
        return x_cam, y_cam, z_cam

    def image_callback(self, color_msg):
        # 保存最新的彩色图像
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            rospy.logdebug("Received new color image")
        except Exception as e:
            rospy.logwarn(f"Failed to convert color image: {e}")
            return

        # 检查深度图像和相机信息是否可用
        if self.latest_depth_image is None or self.latest_camera_info is None:
            rospy.logwarn("Depth image or camera info not available")
            return

        # 获取彩色图像
        cv_image = self.latest_color_image

        target_class = "egg"
        results = self.model([cv_image])
        current_midpoints = []
        base_coordinates = []

        for result in results:
            boxes = result.boxes
            rospy.loginfo(f"Detected {len(boxes)} objects")
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                rospy.loginfo(f"Detected object class: {class_name}")

                if class_name == target_class:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    midpoint_x = (x1 + x2) / 2
                    midpoint_y = (y1 + y2) / 2
                    rospy.loginfo(f"Target '{target_class}' detected at ({midpoint_x}, {midpoint_y})")

                    # 检查像素坐标是否在图像范围内
                    if midpoint_x < 0 or midpoint_x >= self.latest_depth_image.shape[1] or \
                       midpoint_y < 0 or midpoint_y >= self.latest_depth_image.shape[0]:
                        rospy.logwarn("Midpoint coordinates out of depth image bounds")
                        continue

                    # 使用区域平均深度
                    x_min = max(int(midpoint_x) - 2, 0)
                    x_max = min(int(midpoint_x) + 2, self.latest_depth_image.shape[1] - 1)
                    y_min = max(int(midpoint_y) - 2, 0)
                    y_max = min(int(midpoint_y) + 2, self.latest_depth_image.shape[0] - 1)
                    depth_roi = self.latest_depth_image[y_min:y_max+1, x_min:x_max+1]
                    valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < 65535)]
                    if valid_depths.size == 0:
                        rospy.logwarn("No valid depth pixels in ROI")
                        continue
                    depth_value_mm = np.mean(valid_depths)
                    depth_value = depth_value_mm * 0.001  # 转换为米
                    rospy.loginfo(f"Depth at midpoint: {depth_value:.2f} meters")

                    if self.min_depth < depth_value < self.max_depth:
                        x_cam, y_cam, z_cam = self.convert_depth_to_phys_coord_using_realsense(
                            midpoint_x, midpoint_y, depth_value_mm, self.latest_camera_info
                        )
                        current_midpoints.append((x_cam, y_cam, z_cam))
                        rospy.loginfo(f"Camera coordinates: (x={x_cam:.3f}, y={y_cam:.3f}, z={z_cam:.3f})")

                        # 转换到基座坐标系
                        x_base, y_base, z_base = self.transform_camera_to_base(x_cam, y_cam, z_cam)
                        x_base = x_base - 145.341 #标定补偿
                        y_base = - y_base - 200 #标定补偿
                        z_base = z_base + 440 #标定补偿
                        base_coordinates.append((-x_base, -y_base, z_base))
                        rospy.loginfo(f"Base coordinates: (x={x_base:.3f}, y={y_base:.3f}, z={z_base:.3f})")

                        # 绘制边框和深度信息
                        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(cv_image, f'Depth: {depth_value:.2f}m', (int(x1), int(y1)-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        rospy.logwarn(f"Depth {depth_value:.2f} out of range")
                else:
                    rospy.loginfo(f"Ignoring class '{class_name}'")

        with self.midpoints_lock:
            self.midpoints = current_midpoints

        # 发布相机坐标系下的中点信息
        if self.midpoints:
            bbox_msg = Float32MultiArray()
            bbox_msg.data = [coord for midpoint in self.midpoints for coord in midpoint]
            self.bbox_publisher.publish(bbox_msg)
            rospy.loginfo(f"Published camera coordinates: {self.midpoints}")

        # 发布基座坐标系下的物体位置
        if base_coordinates:
            base_coords_msg = Float32MultiArray()
            base_coords_msg.data = [coord for point in base_coordinates for coord in point]
            self.base_coords_publisher.publish(base_coords_msg)
            rospy.loginfo(f"Published base coordinates: {base_coordinates}")

        # 可视化检测结果（如果需要显示）
        # cv2.imshow("YOLO Detection", cv_image)
        # cv2.waitKey(1)

    def print_midpoints(self, event):
        with self.midpoints_lock:
            if self.midpoints:
                rospy.loginfo(f"Midpoints in camera coordinates: {self.midpoints}")

    def shutdown_callback(self):
        rospy.loginfo("Shutting down YOLO detection node...")
        cv2.destroyAllWindows()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YOLODetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
