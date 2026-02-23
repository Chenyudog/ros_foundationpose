import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from ultralytics import YOLO

rgb_path='/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb'
yolomodel_path=""

class HandDetectNode(Node):
    def __init__(self):
        super().__init__('hand_detect_node')#设置节点名称

        # 加载自定义YOLO手部检测模型
        self.model=YOLO(yolomodel_path)

        # 创建图像发布者（发布带检测框的图像）
        self.img_pub=self.create_publisher(
            Image,#消息类型
            "/camera/hand_detect_node",#发布的话题名称
            10
        )

        # 初始化CVBridge
        self.bridge=CvBridge()  
        self.get_logger().info("hand_detect_node Start")

    def img_callback(self,msg):
        try:
            # 1. ROS Image → OpenCV图像
            cv_image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
            
            # 2. YOLO模型推理
            results=self.model(cv_image,conf=0.25)
            
            # 3. 可视化推理结果（绘制检测框、类别、置信度）
            vis_image = results[0].plot()  # YOLO内置可视化函数
            
            # 4. OpenCV图像 → ROS Image消息
            ros_vis_image = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
            # 复用原始图像的时间戳和帧ID（方便同步）
            ros_vis_image.header = msg.header
            
            # 5. 发布带检测框的图像
            self.img_pub.publish(ros_vis_image)

        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败：{e}")
        except Exception as e:
            self.get_logger().error(f"图像处理失败：{e}")

def main(args=None):
    rclpy.init(args=args)  # 保留args参数，兼容命令行传参
    node=HandDetectNode()
    rclpy.spin(node)
    # 销毁节点（释放资源）
    node.destroy_node()
    rclpy.shutdown()

# 程序入口
if __name__ == "__main__":
    main()
