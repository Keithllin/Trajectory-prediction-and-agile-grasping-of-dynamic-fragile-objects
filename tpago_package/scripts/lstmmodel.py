#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from sklearn.preprocessing import MinMaxScaler
import pickle
import matplotlib.pyplot as plt

# 设置随机种子以确保结果可复现
def set_seed(seed):
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)

set_seed(42)

# 设置设备
torch.cuda.set_device(0)  # 如果有多个GPU，可根据需要修改
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 定义LSTM模型
class LSTM(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size):
        super(LSTM, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        # 初始化隐藏状态和细胞状态
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(device)
        
        # 前向传播LSTM
        out, _ = self.lstm(x, (h0, c0))
        
        # 通过全连接层
        out = self.fc(out[:, -1, :])
        return out

# 定义创建序列的函数
def create_sequences(data, seq_length):
    xs, ys = [], []
    for i in range(len(data) - seq_length - 1):
        x = data[i:(i + seq_length)]
        y = data[i + seq_length]  
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)

# 如果此脚本被直接运行，则执行以下代码
if __name__ == "__main__":
    # 加载数据
    file_path = '/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/tpago/src/realsense_example/scripts/trajectories/trajectory_3d_20241203-154346.npy'  
    data_values = np.load(file_path)
    
    # 归一化数据
    scaler = MinMaxScaler(feature_range=(0, 1))
    data_values = scaler.fit_transform(data_values)
    
    # 保存归一化器
    with open('scaler.pkl', 'wb') as f:
        pickle.dump(scaler, f)
    
    # 创建序列
    seq_length = 6
    X, y = create_sequences(data_values, seq_length)
    
    # 划分训练集和测试集
    train_size = int(len(y) * 0.75)
    X_train, X_test = X[:train_size], X[train_size:]
    y_train, y_test = y[:train_size], y[train_size:]
    
    # 转换为PyTorch张量
    X_train = torch.from_numpy(X_train).float().to(device)
    y_train = torch.from_numpy(y_train).float().to(device)
    X_test = torch.from_numpy(X_test).float().to(device)
    y_test = torch.from_numpy(y_test).float().to(device)
    
    # 初始化模型
    input_size = X_train.shape[2]  # 输入特征维度
    hidden_size = 50
    num_layers = 1
    output_size = y_train.shape[1]  # 输出特征维度
    
    model = LSTM(input_size, hidden_size, num_layers, output_size).to(device)
    
    # 设置训练参数
    learning_rate = 0.001
    num_epochs = 200
    
    # 定义损失函数和优化器
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    
    # 训练模型
    for epoch in range(num_epochs):
        model.train()
        outputs = model(X_train)
        optimizer.zero_grad()
        loss = criterion(outputs, y_train)
        loss.backward()
        optimizer.step()
    
        if (epoch + 1) % 10 == 0:
            print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}")
    
    # 保存模型
    torch.save(model.state_dict(), 'lstm_model.pth')
    print("训练完成，模型已保存。")
    
    # 评估模型在测试集上的表现
    model.eval()
    with torch.no_grad():
        test_outputs = model(X_test)
        test_loss = criterion(test_outputs, y_test)
        print(f"Test Loss: {test_loss.item():.4f}")
    
    # 将训练和测试的预测结果反归一化
    train_outputs = model(X_train).detach().cpu().numpy()
    test_outputs = test_outputs.detach().cpu().numpy()
    train_outputs = scaler.inverse_transform(train_outputs)
    test_outputs = scaler.inverse_transform(test_outputs)
    
    # 可视化结果（可选）
    # data_values_orig = scaler.inverse_transform(data_values)
    # plt.figure(figsize=(12, 8))
    # plt.plot(data_values_orig[:, 0], data_values_orig[:, 1], data_values_orig[:, 2], label='True Values', color='blue')
    # plt.plot(test_outputs[:, 0], test_outputs[:, 1], test_outputs[:, 2], label='Test Predictions', color='orange')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('LSTM Predictions vs True Values in 3D Space')
    # plt.legend()
    # plt.show()
