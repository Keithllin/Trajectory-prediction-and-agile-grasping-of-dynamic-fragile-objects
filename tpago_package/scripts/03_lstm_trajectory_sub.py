#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import torch
import numpy as np
import sys
import os
import pickle

# 获取当前脚本所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 将当前目录添加到 sys.path
sys.path.append(current_dir)

from lstmmodel import LSTM  # 确保导入正确的模型定义

class TrajectoryPredictionNode:
    def __init__(self):
        # 初始化模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = LSTM(input_size=3, hidden_size=50, output_size=3, num_layers=1).to(self.device)
        model_path = '/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/tpago/lstm_model.pth'
        if os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            rospy.loginfo("LSTM 模型加载完成")
        else:
            rospy.logerr(f"模型文件未找到: {model_path}")
            rospy.signal_shutdown("模型文件未找到")
            return

        # 加载归一化器
        scaler_path = '/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/tpago/scaler.pkl'
        if os.path.exists(scaler_path):
            with open(scaler_path, 'rb') as f:
                self.scaler = pickle.load(f)
            rospy.loginfo("归一化器已加载")
        else:
            rospy.logerr(f"归一化器文件未找到: {scaler_path}")
            rospy.signal_shutdown("归一化器文件未找到")
            return

        # 预测结果发布者
        self.predicted_positions_pub = rospy.Publisher('/predicted_positions', Float32MultiArray, queue_size=10)
        rospy.loginfo("已创建预测结果发布者 /predicted_positions")

        # 轨迹缓冲区
        self.trajectory_buffer = []
        self.buffer_size = 30  # 根据模型的输入序列长度设置
        rospy.loginfo(f"轨迹缓冲区大小设置为 {self.buffer_size}")

        # UR5 机械臂移动速度，单位：米/秒
        self.ur5_speed = 0.1  # 请根据实际情况调整
        rospy.loginfo(f"UR5 机械臂移动速度设置为 {self.ur5_speed} m/s")

        # 订阅轨迹数据
        rospy.Subscriber('/trajectory_3d', Float32MultiArray, self.trajectory_callback)
        rospy.loginfo("已订阅 /trajectory_3d 话题")

    def trajectory_callback(self, msg):
        data = msg.data
        if not data:
            rospy.logwarn("接收到空的轨迹数据")
            return

        rospy.loginfo(f"接收到轨迹数据：{data}")

        if len(data) != 3:
            rospy.logwarn("接收到的轨迹数据长度不是3，数据可能不完整")
            return

        point = data[:3]
        self.trajectory_buffer.append(point)
        rospy.loginfo(f"当前轨迹缓冲区大小：{len(self.trajectory_buffer)}")

        if len(self.trajectory_buffer) > self.buffer_size:
            self.trajectory_buffer = self.trajectory_buffer[-self.buffer_size:]
            rospy.loginfo("已更新缓冲区，保留最近的轨迹点")

        if len(self.trajectory_buffer) == self.buffer_size:
            rospy.loginfo("缓冲区已满，开始进行预测")

            # 将缓冲区数据转换为 NumPy 数组
            input_trajectory = np.array(self.trajectory_buffer)

            # 对输入数据进行归一化
            input_trajectory_norm = self.scaler.transform(input_trajectory)

            # 转换为张量
            input_trajectory_tensor = torch.tensor(input_trajectory_norm, dtype=torch.float32).unsqueeze(0).to(self.device)

            # 预测下一个位置
            with torch.no_grad():
                predicted_position_norm = self.model(input_trajectory_tensor)
                predicted_position_norm = predicted_position_norm.cpu().numpy().flatten()
                rospy.loginfo(f"归一化的预测位置：{predicted_position_norm}")

                # 对预测结果进行反归一化
                predicted_position = self.scaler.inverse_transform([predicted_position_norm])[0]
                rospy.loginfo(f"反归一化后的预测位置：{predicted_position}")

            # 发布预测结果
            prediction_msg = Float32MultiArray()
            prediction_msg.data = predicted_position.tolist()
            self.predicted_positions_pub.publish(prediction_msg)
            rospy.loginfo("已发布预测结果到 /predicted_positions 话题")

            # 获取当前未归一化的当前位置
            current_position = np.array(self.trajectory_buffer[-1])

            # 计算距离和预计时间
            distance = np.linalg.norm(predicted_position - current_position)
            estimated_time = distance / self.ur5_speed if self.ur5_speed > 0 else 0.0
            rospy.loginfo(f"从当前位置 {current_position} 到预测位置的距离：{distance:.4f} 毫米")
            rospy.loginfo(f"预计 UR5 移动到预测位置需要时间：{estimated_time:.4f} 秒")

            # 清空缓冲区
            self.trajectory_buffer = []

            # 发布完成后关闭节点
            rospy.signal_shutdown("预测完成，节点停止")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('trajectory_prediction_node')
    node = TrajectoryPredictionNode()
    node.run()