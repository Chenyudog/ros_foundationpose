import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
from rclpy.time import Time
import time as t

class PictureSaveNode(Node):
    def __init__(self):
        super().__init__("picture_save_node")
        
        # ========== 核心配置（针对性解决只丢Depth问题） ==========
        self.img_count = 1
        self.count_lock = threading.Lock()
        self.time_threshold = 0.03  # 放宽匹配阈值（30ms），给Depth更多匹配机会
        self.cache_timeout = 2.0    # 延长缓存超时（2s），让RGB多等Depth一会儿
        self.rgb_cache_max = 5      # 限制RGB缓存数量（避免无意义堆积）
        self.depth_cache_max = 15   # 增大Depth缓存（优先保障Depth不丢）
        
        # ========== 缓存设计（Depth优先） ==========
        self.rgb_cache = {}    # {时间戳key: (数据, msg, 缓存时间)}
        self.depth_cache = {}  # Depth缓存更大，优先保留
        self.saved_pairs = set()
        
        # ========== 路径（与你的原路径一致） ==========
        self.rgb_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb"  
        self.depth_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/depth"
        
        # ========== 初始化 ==========
        self.bridge = CvBridge()
        self._create_directory(self.rgb_path)
        self._create_directory(self.depth_path)
        
        # ========== 订阅话题（Depth回调优先级更高） ==========
        # RGB订阅队列调小，避免积压；Depth队列调大，优先接收
        self.rgb_sub = self.create_subscription(
            Image, "camera/color/image_raw", self.rgb_callback, 5)
        self.depth_sub = self.create_subscription(
            Image, "camera/depth/image_raw", self.depth_callback, 20)
        
        # ========== 定时任务（优先清理RGB，保留Depth） ==========
        self.cleanup_timer = self.create_timer(0.3, self._cleanup_cache)  # 更频繁清理
        
        self.get_logger().info("✅ 深度优先版RGB-D保存节点已启动（解决只丢Depth问题）")
        self.get_logger().info(f"RGB缓存上限: {self.rgb_cache_max} | Depth缓存上限: {self.depth_cache_max}")
        self.get_logger().info(f"匹配阈值: {self.time_threshold}s | 缓存超时: {self.cache_timeout}s")

    # 辅助函数：创建目录
    def _create_directory(self, path):
        try:
            os.makedirs(path, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"创建目录失败 {path}: {str(e)}")
            raise

    # 辅助函数：生成时间戳key
    def _get_stamp_key(self, stamp):
        return f"{stamp.sec}.{stamp.nanosec}"

    # 辅助函数：计算时间差
    def _calc_time_diff(self, stamp1, stamp2):
        t1 = Time.from_msg(stamp1).nanoseconds / 1e9
        t2 = Time.from_msg(stamp2).nanoseconds / 1e9
        return abs(t1 - t2)

    # ========== RGB回调（限制缓存，主动等Depth） ==========
    def rgb_callback(self, msg):
        try:
            # 1. 先判断RGB缓存是否已满，满了就丢弃最新的（优先保Depth）
            if len(self.rgb_cache) >= self.rgb_cache_max:
                oldest_key = list(self.rgb_cache.keys())[0]
                del self.rgb_cache[oldest_key]
                self.get_logger().warn(f"⚠️ RGB缓存已满，丢弃最早帧: {oldest_key}（优先保Depth）")
            
            # 2. 转换并缓存RGB（只缓存，不保存）
            rgb_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            stamp_key = self._get_stamp_key(msg.header.stamp)
            self.rgb_cache[stamp_key] = (rgb_data, msg, t.time())
            
            # 3. 主动匹配Depth（只找Depth，不主动触发保存）
            self._match_depth_for_rgb(stamp_key)
            
        except Exception as e:
            self.get_logger().error(f"RGB缓存失败: {str(e)}")

    # ========== Depth回调（优先缓存，主动匹配所有RGB） ==========
    def depth_callback(self, msg):
        try:
            # 1. 转换并缓存Depth（缓存上限更大，优先保留）
            depth_data = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            stamp_key = self._get_stamp_key(msg.header.stamp)
            self.depth_cache[stamp_key] = (depth_data, msg, t.time())
            
            # 2. 关键：Depth到了主动匹配所有缓存的RGB（解决只丢Depth）
            self._match_all_rgb_for_depth(stamp_key)
            
            # 3. 清理Depth超量缓存（上限更高）
            if len(self.depth_cache) > self.depth_cache_max:
                oldest_key = list(self.depth_cache.keys())[0]
                del self.depth_cache[oldest_key]
                self.get_logger().debug(f"清理超量Depth缓存: {oldest_key}")
            
        except Exception as e:
            self.get_logger().error(f"Depth缓存失败: {str(e)}")

    # ========== 核心：Depth主动匹配所有RGB（解决只丢Depth） ==========
    def _match_all_rgb_for_depth(self, depth_stamp_key):
        """Depth帧到达后，遍历所有缓存的RGB，找匹配的帧对（Depth主导匹配）"""
        depth_data, depth_msg, _ = self.depth_cache[depth_stamp_key]
        matched_rgb_key = None
        min_diff = float('inf')
        matched_rgb_data = None

        # 遍历所有RGB缓存，找最匹配的
        for rgb_key, (rgb_data, rgb_msg, _) in self.rgb_cache.items():
            time_diff = self._calc_time_diff(depth_msg.header.stamp, rgb_msg.header.stamp)
            if time_diff < self.time_threshold and time_diff < min_diff:
                min_diff = time_diff
                matched_rgb_key = rgb_key
                matched_rgb_data = rgb_data

        # 找到匹配的RGB：由Depth主导保存（核心改！）
        if matched_rgb_key:
            with self.count_lock:
                current_seq = self.img_count
                self.img_count += 1
            
            # 保存成对帧
            self._save_paired_frames(current_seq, matched_rgb_data, depth_data, f"{rgb_key}+{depth_stamp_key}")
            
            # 删除已匹配的缓存（RGB匹配后立即删除，避免重复）
            del self.rgb_cache[matched_rgb_key]
            del self.depth_cache[depth_stamp_key]

    # ========== 辅助：RGB找Depth（兜底） ==========
    def _match_depth_for_rgb(self, rgb_stamp_key):
        """RGB主动找Depth（兜底，主要匹配逻辑交给Depth）"""
        rgb_data, rgb_msg, _ = self.rgb_cache[rgb_stamp_key]
        matched_depth_key = None
        min_diff = float('inf')
        matched_depth_data = None

        for depth_key, (depth_data, depth_msg, _) in self.depth_cache.items():
            time_diff = self._calc_time_diff(rgb_msg.header.stamp, depth_msg.header.stamp)
            if time_diff < self.time_threshold and time_diff < min_diff:
                min_diff = time_diff
                matched_depth_key = depth_key
                matched_depth_data = depth_data

        if matched_depth_key:
            with self.count_lock:
                current_seq = self.img_count
                self.img_count += 1
            
            self._save_paired_frames(current_seq, rgb_data, matched_depth_data, f"{rgb_stamp_key}+{matched_depth_key}")
            del self.rgb_cache[rgb_stamp_key]
            del self.depth_cache[matched_depth_key]

    # ========== 成对保存（严格保障，Depth优先） ==========
    def _save_paired_frames(self, seq, rgb_data, depth_data, stamp_pair):
        BASE_TS = 1234567890123456789
        new_name = f"{BASE_TS + seq * 1000}"

        rgb_file = os.path.join(self.rgb_path, f"{new_name}.png")
        depth_file = os.path.join(self.depth_path, f"{new_name}.png")

        rgb_ok = False
        depth_ok = False

        # 保存RGB
        try:
            rgb_ok = cv2.imwrite(rgb_file, rgb_data)
        except Exception as e:
            self.get_logger().error(f"序号{seq} RGB保存异常: {str(e)}")

        # 保存Depth（增强容错）
        try:
            if cv2.imwrite(depth_file, depth_data):
                depth_ok = True
            else:
                npy_file = depth_file.replace(".png", ".npy")
                np.save(npy_file, depth_data)
                self.get_logger().warn(f"序号{seq} Depth PNG失败，降级为npy: {npy_file}")
                depth_ok = True
        except Exception as e:
            self.get_logger().error(f"序号{seq} Depth保存异常: {str(e)}")

        # 校验结果（只丢Depth的情况直接回退）
        if rgb_ok and depth_ok:
            self.get_logger().info(f"✅ 序号{seq} | RGB-D保存成功 | 帧对: {stamp_pair}")
            self.saved_pairs.add(stamp_pair)
        else:
            # 重点：只要Depth没保存成功，就清理+回退
            self.get_logger().error(f"❌ 序号{seq} | RGB: {rgb_ok} | Depth: {depth_ok} | 清理文件+回退序号")
            if os.path.exists(rgb_file):
                os.remove(rgb_file)
            if os.path.exists(depth_file):
                os.remove(depth_file)
            with self.count_lock:
                self.img_count -= 1

    # ========== 缓存清理（优先保留Depth，只清理超时RGB） ==========
    def _cleanup_cache(self):
        current_time = t.time()
        # 1. 清理超时RGB（只丢RGB，不丢Depth）
        expired_rgb = [k for k, (_, _, cache_time) in self.rgb_cache.items() 
                       if current_time - cache_time > self.cache_timeout]
        for k in expired_rgb:
            self.get_logger().warn(f"⚠️ 超时清理：RGB帧{k}等待Depth超时，已删除")
            del self.rgb_cache[k]

        # 2. Depth缓存尽量保留，只清理严重超时的（>5s）
        expired_depth = [k for k, (_, _, cache_time) in self.depth_cache.items() 
                         if current_time - cache_time > 5.0]
        for k in expired_depth:
            self.get_logger().warn(f"⚠️ 深度超时清理：Depth帧{k}超过5s无匹配RGB，已删除")
            del self.depth_cache[k]

# 主函数
def main(args=None):
    rclpy.init(args=args)
    node = PictureSaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()