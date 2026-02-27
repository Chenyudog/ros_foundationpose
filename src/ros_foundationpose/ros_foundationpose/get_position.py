import rclpy
from rclpy.node import Node
import os
import glob

position_path="/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/position.txt"

class GetPositionNode(Node):
    def __init__(self):
        super().__init__("GetPositionNode")
        

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    # 创建节点（创建即执行清空操作）
    node = GetPositionNode()
    # 自旋直到节点退出（仅为触发定时器，不会持续运行）
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()