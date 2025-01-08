from ultralytics import YOLO

# 加载预训练模型（例如 YOLOv8n）
model = YOLO('yolov8n.pt')

# 训练模型
results = model.train(
    data='/home/hammer/YOLO_training_data/handler/data.yaml',  # 数据集配置文件
    epochs=100,        # 训练轮数
    imgsz=640,         # 输入图片尺寸
    batch=16,          # 批次大小
    name='yolo_handler' # 训练结果保存名称
)
