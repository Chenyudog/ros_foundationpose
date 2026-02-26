import cv2
import numpy as np
import os
import time as t
from ultralytics import YOLO  # 新增：YOLO依赖

# ========== YOLO配置（集中管理） ==========
YOLO_MODEL_PATH = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/yolomodel/best.pt"  # 补全权重文件（通常是best.pt）
MASK_SAVE_PATH = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/masks"
TARGET_CLASS_ID = 0  # 手部类别ID（根据你的YOLO模型调整）

# 初始化YOLO模型（全局只加载一次，避免重复加载）
try:
    yolo_model = YOLO(YOLO_MODEL_PATH)
    # 创建掩码保存目录
    os.makedirs(MASK_SAVE_PATH, exist_ok=True)
except Exception as e:
    raise Exception(f"YOLO模型加载/掩码目录创建失败: {str(e)}")

# ========== 生成手部掩码（核心函数，适配Seg模型） ==========
def generate_hand_mask(rgb_image, mask_filename):
    """
    生成手部掩码图（白手黑背景）
    :param rgb_image: OpenCV格式的RGB图像
    :param mask_filename: 掩码保存的文件名（和RGB/Depth同名）
    :return: (是否成功, 成功返回保存路径/失败返回错误信息)
    """
    try:
        # 1. YOLO Seg模型推理（分割模型会输出masks）
        results = yolo_model(rgb_image, conf=0.6)
        result = results[0]  # 取第一张图的推理结果
        
        # 2. 生成纯黑掩码（和原图同尺寸）
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        
        # 3. 遍历分割结果（适配Seg模型的masks接口）
        if result.masks is not None:
            # 遍历每个分割掩码
            for i, seg_mask in enumerate(result.masks.data):
                # 获取当前掩码对应的类别ID和置信度
                cls_id = int(result.boxes.cls[i])
                conf = float(result.boxes.conf[i])
                
                # 只保留目标类别（手部）且置信度符合要求的掩码
                if cls_id == TARGET_CLASS_ID and conf > 0.25:
                    # 将tensor格式的掩码转为numpy数组，并调整尺寸和原图一致
                    seg_mask = seg_mask.cpu().numpy()  # 转到CPU并转为numpy
                    seg_mask = cv2.resize(seg_mask, (rgb_image.shape[1], rgb_image.shape[0]))  # 匹配原图尺寸
                    # 将分割区域设为白色（255）
                    mask[seg_mask > 0.9] = 255  # 分割掩码是0-1的概率值，>0.5视为前景
        
        # 4. 保存掩码图（和RGB/Depth同名）
        mask_save_path = os.path.join(MASK_SAVE_PATH, mask_filename)
        cv2.imwrite(mask_save_path, mask)
        return True, mask_save_path
    except Exception as e:
        return False, str(e)

# ========== 测试代码（可选，验证功能） ==========
if __name__ == "__main__":
    # 测试用例：读取一张RGB图，生成掩码
    test_rgb_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb/test.jpg"
    if os.path.exists(test_rgb_path):
        rgb_img = cv2.imread(test_rgb_path)
        success, res = generate_hand_mask(rgb_img, "test_mask.png")
        if success:
            print(f"掩码生成成功，保存路径：{res}")
        else:
            print(f"掩码生成失败：{res}")
    else:
        print(f"测试图片不存在：{test_rgb_path}")