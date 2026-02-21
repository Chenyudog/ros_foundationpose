import cv2
import numpy as np
import json
import os
import glob
os.environ['QT_QPA_PLATFORM'] = 'offscreen'  # 强制OpenCV跳过Qt，用纯后端

def create_mask_from_json(json_path, rgb_path, output_path):
    """
    根据 labelme 的 .json 文件、对应的RGB图像路径，
    创建一个黑底白前景的蒙版图像。
    (此函数无需修改)
    """

    # 1. 读取RGB图像，只为了获取 [高度, 宽度]
    image = cv2.imread(rgb_path)
    if image is None:
        print(f"[警告] 无法读取RGB图像: {rgb_path}。跳过此文件。")
        return False

    height, width = image.shape[:2]

    # 2. 创建一个全黑的空白画布（蒙版）
    mask = np.zeros((height, width), dtype=np.uint8)

    # 3. 读取并解析 JSON 文件
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[错误] 无法读取或解析JSON: {json_path}。错误: {e}")
        return False

    # 4. 遍历 JSON 中所有的 'shapes'（你画的标注）
    polygons = []
    for shape in data['shapes']:
        # 获取多边形的所有点
        points = np.array(shape['points'], dtype=np.int32)
        polygons.append(points)

    if not polygons:
        print(f"[警告] {os.path.basename(json_path)} 中未找到标注 'shapes'。将保存一张全黑蒙版。")

    # 5. 在黑色蒙版上将所有多边形区域填充为白色（255）
    cv2.fillPoly(mask, polygons, (255))

    # 6. 保存这张蒙版图像
    try:
        cv2.imwrite(output_path, mask)
        return True
    except Exception as e:
        print(f"[错误] 无法保存蒙版: {output_path}。错误: {e}")
        return False

def main():
    # -------------------------------------------------------------------------
    # --- (修改) ---
    # -------------------------------------------------------------------------

    # 1. 获取脚本所在的目录
    # 假设你的脚本、JSON 和 RGB 文件都在这个目录
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
    except NameError:
        script_dir = r"/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/masks"  # Fallback 路径

    # 数据目录就是脚本目录
    data_dir = script_dir

    print("--- 开始生成蒙版 ---")
    print(f"处理目录（脚本，JSON，RGB）: {data_dir}")

    # 2. 查找当前目录中所有的 .json 文件
    json_files = glob.glob(os.path.join(data_dir, "*.json"))

    if not json_files:
        print(f"\n错误: 在 '{data_dir}' 文件夹中未找到任何 .json 文件。")
        return

    print(f"\n找到了 {len(json_files)} 个 .json 文件，开始处理...")

    processed_count = 0
    skipped_count = 0

    # 3. 遍历所有找到的 JSON 文件
    for json_path in json_files:

        # 'frame_...316545.json' -> 'frame_...316545'
        file_stem = os.path.splitext(os.path.basename(json_path))[0]

        print(f"\n处理: {file_stem}.json")

        # 4. 自动查找对应的 .png 或 .jpg 文件
        rgb_path_png = os.path.join(data_dir, file_stem + ".png")
        rgb_path_jpg = os.path.join(data_dir, file_stem + ".jpg")

        rgb_path = None
        if os.path.exists(rgb_path_png):
            rgb_path = rgb_path_png
        elif os.path.exists(rgb_path_jpg):
            rgb_path = rgb_path_jpg
        else:
            print(f"[警告] 找不到对应的RGB文件 ({file_stem}.png 或 {file_stem}.jpg)。跳过。")
            skipped_count += 1
            continue

        # 5. 构建输出掩码的路径（在同一目录）
        output_mask_path = os.path.join(data_dir, file_stem + "_mask.png")

        # 6. 调用函数生成蒙版
        if create_mask_from_json(json_path, rgb_path, output_mask_path):
            print(f"[成功] 已保存蒙版: {output_mask_path}")
            processed_count += 1
        else:
            skipped_count += 1

    # --- (修改结束) ---
    # -------------------------------------------------------------------------

    print("\n--- 处理完毕 ---")
    print(f"成功生成: {processed_count} 个蒙版")
    print(f"跳过/失败: {skipped_count} 个文件")


if __name__ == "__main__":
    main()