# utils/take_photo.py
import cv2
import numpy as np
import os
import time as t

from ros_foundationpose.utils import YOLO as yolo_utils  # 正确
# ========== 核心：Depth主动匹配所有RGB ==========
def match_all_rgb_for_depth(node, depth_stamp_key):
    depth_data, depth_msg, _ = node.depth_cache[depth_stamp_key]
    matched_rgb_key = None
    min_diff = float('inf')
    matched_rgb_data = None

    for rgb_key, (rgb_data, rgb_msg, _) in node.rgb_cache.items():
        time_diff = node._calc_time_diff(depth_msg.header.stamp, rgb_msg.header.stamp)
        if time_diff < node.time_threshold and time_diff < min_diff:
            min_diff = time_diff
            matched_rgb_key = rgb_key
            matched_rgb_data = rgb_data

    if matched_rgb_key:
        with node.count_lock:
            current_seq = node.img_count
            node.img_count += 1
        
        # 保存RGB-D并生成掩码
        save_paired_frames(node, current_seq, matched_rgb_data, depth_data, f"{rgb_key}+{depth_stamp_key}")
        
        del node.rgb_cache[matched_rgb_key]
        del node.depth_cache[depth_stamp_key]

# ========== 辅助：RGB找Depth ==========
def match_depth_for_rgb(node, rgb_stamp_key):
    rgb_data, rgb_msg, _ = node.rgb_cache[rgb_stamp_key]
    matched_depth_key = None
    min_diff = float('inf')
    matched_depth_data = None

    for depth_key, (depth_data, depth_msg, _) in node.depth_cache.items():
        time_diff = node._calc_time_diff(rgb_msg.header.stamp, depth_msg.header.stamp)
        if time_diff < node.time_threshold and time_diff < min_diff:
            min_diff = time_diff
            matched_depth_key = depth_key
            matched_depth_data = depth_data

    if matched_depth_key:
        with node.count_lock:
            current_seq = node.img_count
            node.img_count += 1
        
        save_paired_frames(node, current_seq, rgb_data, matched_depth_data, f"{rgb_stamp_key}+{matched_depth_key}")
        del node.rgb_cache[rgb_stamp_key]
        del node.depth_cache[matched_depth_key]

# ========== 成对保存RGB-D + 生成掩码 ==========
def save_paired_frames(node, seq, rgb_data, depth_data, stamp_pair):
    BASE_TS = 1234567890123456789
    new_name = f"{BASE_TS + seq * 1000}.png"  # 明确后缀为.png

    rgb_file = os.path.join(node.rgb_path, new_name)
    depth_file = os.path.join(node.depth_path, new_name)

    rgb_ok = False
    depth_ok = False

    # 保存RGB
    try:
        rgb_ok = cv2.imwrite(rgb_file, rgb_data)
    except Exception as e:
        node.get_logger().error(f"序号{seq} RGB保存异常: {str(e)}")

    # 保存Depth
    try:
        if cv2.imwrite(depth_file, depth_data):
            depth_ok = True
        else:
            npy_file = depth_file.replace(".png", ".npy")
            np.save(npy_file, depth_data)
            node.get_logger().warn(f"序号{seq} Depth PNG失败，降级为npy: {npy_file}")
            depth_ok = True
    except Exception as e:
        node.get_logger().error(f"序号{seq} Depth保存异常: {str(e)}")

    # 校验结果：RGB-D保存成功后，立即生成掩码
    if rgb_ok and depth_ok:
        node.get_logger().info(f"✅ 序号{seq} | RGB-D保存成功 | 帧对: {stamp_pair}")
        node.saved_pairs.add(stamp_pair)
        
        # ========== 核心新增：生成手部掩码 ==========
        mask_ok, mask_info = yolo_utils.generate_hand_mask(rgb_data, new_name)
        if mask_ok:
            node.get_logger().info(f"✅ 序号{seq} | 掩码生成成功: {mask_info}")
        else:
            node.get_logger().error(f"❌ 序号{seq} | 掩码生成失败: {mask_info}")
    else:
        node.get_logger().error(f"❌ 序号{seq} | RGB: {rgb_ok} | Depth: {depth_ok} | 清理文件+回退序号")
        if os.path.exists(rgb_file):
            os.remove(rgb_file)
        if os.path.exists(depth_file):
            os.remove(depth_file)
        with node.count_lock:
            node.img_count -= 1

# ========== 缓存清理 ==========
def cleanup_cache(node):
    current_time = t.time()
    expired_rgb = [k for k, (_, _, cache_time) in node.rgb_cache.items() 
                    if current_time - cache_time > node.cache_timeout]
    for k in expired_rgb:
        node.get_logger().warn(f"⚠️ 超时清理：RGB帧{k}等待Depth超时，已删除")
        del node.rgb_cache[k]

    expired_depth = [k for k, (_, _, cache_time) in node.depth_cache.items() 
                        if current_time - cache_time > 5.0]
    for k in expired_depth:
        node.get_logger().warn(f"⚠️ 深度超时清理：Depth帧{k}超过5s无匹配RGB，已删除")
        del node.depth_cache[k]