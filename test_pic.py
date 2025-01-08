from ultralytics import YOLO

# 加载训练好的模型
model = YOLO('runs/detect/yolo_handler6/weights/best.pt')

# 对验证集进行推理
results = model.predict(source='/home/hammer/YOLO_training_data/handler/val/images/image_92.jpg', save=True, conf=0.25)

# 可视化结果
for result in results:
    result.show()  # 显示检测结果
