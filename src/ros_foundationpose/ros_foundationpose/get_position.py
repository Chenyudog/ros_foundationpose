import rclpy
from rclpy.node import Node
import os
import glob
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import Pose  # 可选：使用ROS标准Pose消息

# 文件路径配置
POSITION_PATH = "/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/debug/ob_in_cam"

class GetPositionNode(Node):
    def __init__(self):
        super().__init__("get_position_node")
        
        # 初始化变量
        self.publish_interval = 0.1  # 发布间隔（秒），10Hz
        
        # 1. 创建发布者1：发布格式化的x y z r p y字符串（易读）
        self.euler_publisher = self.create_publisher(
            String, 
            "/object_pose_euler", 
            10
        )
        
        # 2. 创建发布者2：发布ROS标准Pose消息（便于其他节点使用）
        self.pose_publisher = self.create_publisher(
            Pose, 
            "/object_pose", 
            10
        )
        
        # 创建定时器，按固定频率持续发布
        self.timer = self.create_timer(
            self.publish_interval, 
            self.continuous_publish_pose
        )
        
        self.get_logger().info(
            f"GetPositionNode已启动，以{1/self.publish_interval}Hz解析齐次矩阵并发布欧拉角"
        )

    def get_latest_txt_file(self):
        """获取指定目录下最新的.txt文件"""
        try:
            file_pattern = os.path.join(POSITION_PATH, "*.txt")
            txt_files = glob.glob(file_pattern)
            
            if not txt_files:
                self.get_logger().warn("未找到任何txt文件: %s" % file_pattern)
                return None
            
            latest_file = max(txt_files, key=os.path.getmtime)
            return latest_file
        
        except Exception as e:
            self.get_logger().error("获取最新文件失败: %s" % str(e))
            return None

    def parse_homogeneous_matrix(self, file_content):
        """
        解析txt文件中的4x4齐次变换矩阵，返回平移和旋转矩阵
        支持的矩阵格式：空格/换行分隔的16个数值，或4行4列的矩阵
        """
        try:
            # 清理文本，提取所有数字
            numbers = []
            # 按空格、换行、制表符分割，过滤空字符串
            tokens = file_content.replace('\n', ' ').replace('\t', ' ').split()
            for token in tokens:
                try:
                    numbers.append(float(token))
                except ValueError:
                    continue
            
            # 检查是否是4x4矩阵（16个元素）
            if len(numbers) != 16:
                self.get_logger().error(f"矩阵元素数量错误，需16个，实际{len(numbers)}个")
                return None, None
            
            # 转换为4x4 numpy矩阵
            homo_matrix = np.array(numbers).reshape(4, 4)
            
            # 提取平移分量 (x, y, z)
            translation = homo_matrix[:3, 3]  # [x, y, z]
            
            # 提取旋转矩阵 (3x3)
            rotation_matrix = homo_matrix[:3, :3]
            
            return translation, rotation_matrix
        
        except Exception as e:
            self.get_logger().error(f"解析齐次矩阵失败: {str(e)}")
            return None, None

    def rotation_matrix_to_euler(self, rotation_matrix):
        """
        将3x3旋转矩阵转换为欧拉角（rpy，单位：弧度）
        r: roll (绕x轴)
        p: pitch (绕y轴)
        y: yaw (绕z轴)
        """
        try:
            # 使用scipy计算欧拉角（默认顺序：xyz，即rpy）
            r = R.from_matrix(rotation_matrix)
            euler_angles = r.as_euler('xyz', degrees=False)  # 弧度
            roll, pitch, yaw = euler_angles
            return roll, pitch, yaw
        
        except Exception as e:
            self.get_logger().error(f"旋转矩阵转欧拉角失败: {str(e)}")
            return None, None, None

    def read_and_parse_file(self):
        """读取文件并解析为x y z r p y"""
        # 获取最新文件
        latest_file = self.get_latest_txt_file()
        if not latest_file:
            return None
        
        # 读取文件内容
        try:
            with open(latest_file, 'r', encoding='utf-8') as f:
                file_content = f.read().strip()
        except Exception as e:
            self.get_logger().error(f"读取文件失败 {latest_file}: {str(e)}")
            return None
        
        # 解析齐次矩阵
        translation, rotation_matrix = self.parse_homogeneous_matrix(file_content)
        if translation is None or rotation_matrix is None:
            return None
        
        # 转换为欧拉角
        roll, pitch, yaw = self.rotation_matrix_to_euler(rotation_matrix)
        if roll is None:
            return None
        
        # 整理结果
        pose_data = {
            'x': translation[0],
            'y': translation[1],
            'z': translation[2],
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'file_path': latest_file
        }
        
        return pose_data

    def continuous_publish_pose(self):
        """持续解析并发布位姿数据（x y z r p y）"""
        pose_data = self.read_and_parse_file()
        if not pose_data:
            return
        
        # 1. 发布格式化的字符串消息（易读）
        euler_msg = String()
        euler_msg.data = (
            f"x: {pose_data['x']:.6f}, y: {pose_data['y']:.6f}, z: {pose_data['z']:.6f}, "
            f"roll: {pose_data['roll']:.6f}, pitch: {pose_data['pitch']:.6f}, yaw: {pose_data['yaw']:.6f}"
        )
        self.euler_publisher.publish(euler_msg)
        
        # 2. 发布ROS标准Pose消息（便于其他节点使用）
        pose_msg = Pose()
        pose_msg.position.x = pose_data['x']
        pose_msg.position.y = pose_data['y']
        pose_msg.position.z = pose_data['z']
        
        # 将欧拉角转换为四元数（Pose消息要求四元数）
        r = R.from_euler('xyz', [pose_data['roll'], pose_data['pitch'], pose_data['yaw']])
        quat = r.as_quat()  # [x, y, z, w]
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]
        
        self.pose_publisher.publish(pose_msg)
        
        # 打印日志（debug级别避免刷屏）
        self.get_logger().debug(
            f"发布位姿（文件：{os.path.basename(pose_data['file_path'])}）:\n"
            f"x: {pose_data['x']:.6f}, y: {pose_data['y']:.6f}, z: {pose_data['z']:.6f}\n"
            f"roll: {pose_data['roll']:.6f} rad ({np.degrees(pose_data['roll']):.2f}°), "
            f"pitch: {pose_data['pitch']:.6f} rad ({np.degrees(pose_data['pitch']):.2f}°), "
            f"yaw: {pose_data['yaw']:.6f} rad ({np.degrees(pose_data['yaw']):.2f}°)"
        )

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 检查依赖
    try:
        import scipy  # 确保scipy已安装
    except ImportError:
        print("错误：未安装scipy，请执行：pip3 install scipy numpy")
        return
    
    # 创建节点
    node = GetPositionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()