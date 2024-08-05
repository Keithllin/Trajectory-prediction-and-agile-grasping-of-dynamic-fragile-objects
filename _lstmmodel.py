import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from sklearn.preprocessing import MinMaxScaler
import pickle
import matplotlib.pyplot as plt

# random seed
def set_seed(seed):
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)

set_seed(42)

torch.cuda.set_device(0)  #
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Load the NPY data
file_path = '/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/DroneFlightData/data/data_13.npy'  
data_values = np.load(file_path)

#  Normalize data
scaler = MinMaxScaler(feature_range=(0, 1))
data_values = scaler.fit_transform(data_values)

# save scaler
with open('scaler.pkl', 'wb') as f:
    pickle.dump(scaler, f)

# Define a function to create sequences
def create_sequences(data, seq_length):
    xs, ys = [], []
    for i in range(len(data) - seq_length - 1):
        x = data[i:(i + seq_length)]
        y = data[i + seq_length]  
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)

# Set sequence length and preprocess the data
seq_length = 6
X, y = create_sequences(data_values, seq_length)

# Split the data into training and testing sets
train_size = int(len(y) * 0.75)
X_train, X_test = X[:train_size], X[train_size:]
y_train, y_test = y[:train_size], y[train_size:]

# Convert to PyTorch tensors
X_train = torch.from_numpy(X_train).float().to(device)
y_train = torch.from_numpy(y_train).float().to(device)
X_test = torch.from_numpy(X_test).float().to(device)
y_test = torch.from_numpy(y_test).float().to(device)

# Define the LSTM model
class LSTM(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size):
        super(LSTM, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(device)
        
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

# Initialize the LSTM model
input_size = X_train.shape[2]  # input feature dimension
hidden_size = 50
num_layers = 1
output_size = y_train.shape[1]  # output feature dimension

model = LSTM(input_size, hidden_size, num_layers, output_size).to(device)

# Set training parameters
learning_rate = 0.001
num_epochs = 200

# Define loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# Train the model
for epoch in range(num_epochs):
    model.train()
    outputs = model(X_train)
    optimizer.zero_grad()
    loss = criterion(outputs, y_train)
    loss.backward()
    optimizer.step()

    if (epoch + 1) % 10 == 0:
        print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}")

# save model
torch.save(model.state_dict(), 'lstm_model.pth')

# Evaluate the model on the test set
model.eval()
with torch.no_grad():
    test_outputs = model(X_test)
    test_loss = criterion(test_outputs, y_test)
    print(f"Test Loss: {test_loss.item():.4f}")

# Concatenate the training and test predictions
train_outputs = model(X_train).detach().cpu().numpy()
test_outputs = test_outputs.detach().cpu().numpy()
train_outputs = scaler.inverse_transform(train_outputs)
test_outputs = scaler.inverse_transform(test_outputs)

# Plot the results
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# real values
data_values_orig = scaler.inverse_transform(data_values)
ax.plot(data_values_orig[:, 0], data_values_orig[:, 1], data_values_orig[:, 2], label="True Values", color='b')

# Predicted values
all_outputs = np.concatenate((train_outputs, test_outputs))
# ax.plot(train_outputs[:, 0], train_outputs[:, 1], train_outputs[:, 2], label="Train Predictions", color='g')
ax.plot(test_outputs[:, 0], test_outputs[:, 1], test_outputs[:, 2], label="Test Predictions", color='y')
# ax.plot(all_outputs[:, 0], all_outputs[:, 1], all_outputs[:, 2], label="Predictions", color='r')

ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
ax.set_zlabel('Altitude')
ax.legend()
ax.set_title('LSTM Predictions vs True Values in 3D Space')

plt.show()

# # Prediction function for new data
# def lstm_predict(model, input_data):
#     model.eval()
#     input_data = torch.tensor(input_data).float().unsqueeze(0).to(device)
#     with torch.no_grad():
#         prediction = model(input_data).squeeze().cpu().numpy()
#     return prediction
