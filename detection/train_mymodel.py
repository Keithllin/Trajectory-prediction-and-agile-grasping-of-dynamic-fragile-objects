from ultralytics import YOLO

# Load a model # 三选一
model = YOLO('yolov8n.yaml')  # build a new model from YAML
model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)
model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# Train the model
model.train(data='/home/keithlin/dc/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects/ultralytics/ultralytics/cfg/datasets/taskobject.yaml', epochs=100, imgsz=(640,480),device = '0')

