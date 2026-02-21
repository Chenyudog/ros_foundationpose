import rclpy
from rclpy.node import Node
import os
import glob

class ClearImageFolderNode(Node):
    def __init__(self):
        super().__init__("delete_photo_node")
        
        # ========== 配置你要清空的文件夹路径（与保存节点路径一致） ==========
        self.rgb_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb"  
        self.depth_path = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/depth"
        
        # ========== 要删除的文件格式（避免误删其他文件） ==========
        self.delete_extensions = [".png", ".npy"]  # 包含Depth降级保存的npy格式
        
        # 初始化时立即执行清空操作
        self.clear_folders()
        
        # 清空完成后打印日志，5秒后自动退出（给你看日志的时间）
        self.get_logger().info("✅ 清空操作完成，节点即将退出！")
        self.create_timer(5.0, self.shutdown_node)  # 5秒后触发退出

    def clear_folders(self):
        """核心函数：清空指定文件夹内的目标格式文件"""
        # 1. 清空RGB文件夹
        self._clear_single_folder(self.rgb_path, "RGB")
        # 2. 清空Depth文件夹
        self._clear_single_folder(self.depth_path, "Depth")

    def _clear_single_folder(self, folder_path, folder_name):
        """辅助函数：清空单个文件夹，带日志输出"""
        # 检查文件夹是否存在
        if not os.path.exists(folder_path):
            self.get_logger().warn(f"⚠️ {folder_name}文件夹不存在: {folder_path}，已自动创建")
            os.makedirs(folder_path, exist_ok=True)
            return
        
        # 遍历并删除指定格式的文件
        deleted_count = 0
        for ext in self.delete_extensions:
            # 匹配该格式的所有文件
            file_pattern = os.path.join(folder_path, f"*{ext}")
            files_to_delete = glob.glob(file_pattern)
            
            for file_path in files_to_delete:
                try:
                    os.remove(file_path)
                    deleted_count += 1
                    self.get_logger().debug(f"删除{folder_name}文件: {file_path}")  # 调试日志，可关闭
                except Exception as e:
                    self.get_logger().error(f"❌ 删除{folder_name}文件失败 {file_path}: {str(e)}")
        
        # 打印清空结果
        self.get_logger().info(f"✅ {folder_name}文件夹清空完成 | 共删除{deleted_count}个文件 | 路径: {folder_path}")

    def shutdown_node(self):
        """退出节点的函数"""
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    # 创建节点（创建即执行清空操作）
    node = ClearImageFolderNode()
    # 自旋直到节点退出（仅为触发定时器，不会持续运行）
    rclpy.spin(node)

if __name__ == "__main__":
    main()