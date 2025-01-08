from ultralytics import YOLO
import cv2
import numpy as np

# 加载训练好的模型
model = YOLO('runs/detect/yolo_handler6/weights/best.pt')

# 打开摄像头
cap = cv2.VideoCapture(0)  # 0 表示默认摄像头，如果有多个摄像头可以尝试 1, 2, 等

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 设置摄像头分辨率（可选）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 卡尔曼滤波器类
class KalmanFilter:
    def __init__(self):
        # 状态向量 [x, y, vx, vy]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)  # 测量矩阵
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)  # 状态转移矩阵
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.1  # 过程噪声协方差矩阵

    def predict(self, x=None, y=None):
        if x is not None and y is not None:
            # 如果有测量值，更新卡尔曼滤波器
            measured = np.array([[np.float32(x)], [np.float32(y)]])
            self.kf.correct(measured)
        # 预测目标位置
        predicted = self.kf.predict()
        return predicted[0], predicted[1]

# 初始化卡尔曼滤波器字典
kf_dict = {}

while True:
    # 读取摄像头帧
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头帧")
        break

    # 使用 YOLO 进行推理
    results = model.predict(source=frame, conf=0.25)
    
    # 获取当前帧中检测到的目标 ID
    detected_ids = set()
    
    # 遍历检测结果
    for result in results:
        boxes = result.boxes  # 获取检测框信息
        for box in boxes:
            # 获取目标位置
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            
            # 获取目标 ID
            track_id = int(box.id) if box.id is not None else None
            
            # 如果目标 ID 不存在，初始化卡尔曼滤波器
            if track_id not in kf_dict:
                kf_dict[track_id] = KalmanFilter()
            
            # 使用卡尔曼滤波器更新目标位置
            predicted_x, predicted_y = kf_dict[track_id].predict(x_center, y_center)
            
            # 记录当前帧中检测到的目标 ID
            detected_ids.add(track_id)
            
            # 在帧上绘制预测结果
            cv2.circle(frame, (int(predicted_x), int(predicted_y)), 5, (0, 255, 0), -1)  # 绘制预测点
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # 绘制检测框
    
    # 处理被遮挡的目标
    for track_id in list(kf_dict.keys()):
        if track_id not in detected_ids:
            # 如果目标在当前帧中未被检测到，使用卡尔曼滤波器预测其位置
            predicted_x, predicted_y = kf_dict[track_id].predict()
            cv2.circle(frame, (int(predicted_x), int(predicted_y)), 5, (0, 0, 255), -1)  # 绘制预测点（红色表示被遮挡）
    
    # 在帧上绘制检测结果
    annotated_frame = results[0].plot()

    # 显示带检测结果的帧
    cv2.imshow("YOLO Detection", annotated_frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()
