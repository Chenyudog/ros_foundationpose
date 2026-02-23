import cv2
import numpy as np
import os
import time as t
from ultralytics import YOLO  # 新增：YOLO依赖

# ========== YOLO配置（集中管理） ==========
YOLO_MODEL_PATH = "/home/ubuntu/yolo/ultralytics/runs/train/hand_yolov88/weights/best.pt"  # 补全权重文件（通常是best.pt）
MASK_SAVE_PATH = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/masks"
TARGET_CLASS_ID = 0  # 手部类别ID（根据你的YOLO模型调整）

# 初始化YOLO模型（全局只加载一次，避免重复加载）
try:
    yolo_model = YOLO(YOLO_MODEL_PATH)
    # 创建掩码保存目录
    os.makedirs(MASK_SAVE_PATH, exist_ok=True)
except Exception as e:
    raise Exception(f"YOLO模型加载/掩码目录创建失败: {str(e)}")

# ========== 生成手部掩码（核心函数） ==========
def generate_hand_mask(rgb_image, mask_filename):
    """
    生成手部掩码图（白手黑背景）
    :param rgb_image: OpenCV格式的RGB图像
    :param mask_filename: 掩码保存的文件名（和RGB/Depth同名）
    """
    try:
        # 1. YOLO模型推理
        results = yolo_model(rgb_image, conf=0.25)
        
        # 2. 生成纯黑掩码，手部区域设为白色
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        for box in results[0].boxes:
            if box.conf[0] > 0.25 and int(box.cls[0]) == TARGET_CLASS_ID:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                mask[y1:y2, x1:x2] = 255
        
        # 3. 保存掩码图（和RGB/Depth同名）
        mask_save_path = os.path.join(MASK_SAVE_PATH, mask_filename)
        cv2.imwrite(mask_save_path, mask)
        return True, mask_save_path
    except Exception as e:
        return False, str(e)