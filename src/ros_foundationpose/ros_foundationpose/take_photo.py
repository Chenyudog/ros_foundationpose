# 主文件（比如picture_save_node.py）
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
# 导入utils中的核心函数
from ros_foundationpose.utils import take_photo as photo_utils

class PictureSaveNode(Node):
    def __init__(self):
        super().__init__("picture_save_node")
        
        # ========== 核心配置 ==========
        self.img_count = 1
        self.count_lock = threading.Lock()
        self.time_threshold = 0.03  
        self.cache_timeout = 2.0    
        self.rgb_cache_max = 5      
        self.depth_cache_max = 15   
        
        # ========== 缓存设计 ==========
        self.rgb_cache = {}    
        self.depth_cache = {}  
        self.saved_pairs = set()
        
        # ========== 路径 ==========
        self.rgb_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb"  
        self.depth_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/depth"
        
        # ========== 初始化 ==========
        self.bridge = CvBridge()
        self._create_directory(self.rgb_path)
        self._create_directory(self.depth_path)
        
        # ========== 订阅话题 ==========
        self.rgb_sub = self.create_subscription(
            Image, 
            "camera/color/image_raw", 
            self.rgb_callback, 
            5
        )
        
        self.depth_sub = self.create_subscription(
            Image, 
            "camera/depth/image_raw", 
            self.depth_callback, 
            20
        )
        
        # ========== 定时任务（调用utils的清理函数） ==========
        self.cleanup_timer = self.create_timer(0.3, lambda: photo_utils.cleanup_cache(self))  
        
        self.get_logger().info("✅ 深度优先版RGB-D保存节点已启动")
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

    # ========== RGB回调 ==========
    def rgb_callback(self, msg):
        try:
            # 1. 缓存满则丢弃最早帧
            if len(self.rgb_cache) >= self.rgb_cache_max:
                oldest_key = list(self.rgb_cache.keys())[0]
                del self.rgb_cache[oldest_key]
                self.get_logger().warn(f"⚠️ RGB缓存已满，丢弃最早帧: {oldest_key}")
            
            # 2. 转换并缓存RGB
            rgb_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            stamp_key = self._get_stamp_key(msg.header.stamp)
            self.rgb_cache[stamp_key] = (rgb_data, msg, t.time())
            
            # 3. 调用utils的匹配函数（RGB找Depth）
            photo_utils.match_depth_for_rgb(self, stamp_key)
            
        except Exception as e:
            self.get_logger().error(f"RGB缓存失败: {str(e)}")

    # ========== Depth回调 ==========
    def depth_callback(self, msg):
        try:
            # 1. 转换并缓存Depth
            depth_data = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            stamp_key = self._get_stamp_key(msg.header.stamp)
            self.depth_cache[stamp_key] = (depth_data, msg, t.time())
            
            # 2. 调用utils的核心匹配函数（Depth找所有RGB）
            photo_utils.match_all_rgb_for_depth(self, stamp_key)
            
            # 3. 清理超量Depth缓存
            if len(self.depth_cache) > self.depth_cache_max:
                oldest_key = list(self.depth_cache.keys())[0]
                del self.depth_cache[oldest_key]
                self.get_logger().debug(f"清理超量Depth缓存: {oldest_key}")
            
        except Exception as e:
            self.get_logger().error(f"Depth缓存失败: {str(e)}")

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