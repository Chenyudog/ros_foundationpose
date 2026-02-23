import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
from ultralytics import YOLO
import os  # 新增：用于创建目录
import numpy as np  # 新增：用于生成掩码图

rgb_path='/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/rgb'
yolomodel_path="/home/ubuntu/yolo/ultralytics/runs/train/hand_yolov88/weights"
output_mask_path ="/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/demo_data/energy_lattice/masks"

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
        
        # 新增：创建掩码保存目录（exist_ok=True避免重复创建报错）
        os.makedirs(output_mask_path, exist_ok=True)
        
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

            # -------------------------- 新增部分开始 --------------------------
            # 6. 生成手部掩码图（白手黑背景）
            mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)  # 创建纯黑掩码
            for box in results[0].boxes:
                # 筛选置信度>0.25的手部目标
                if box.conf[0] > 0.25 and int(box.cls[0]) == TARGET_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取手部检测框坐标
                    mask[y1:y2, x1:x2] = 255  # 手部区域设为白色
            
            # 7. 保存掩码图（用时间戳命名，和原始图片对应）
            mask_filename = f"mask_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.png"
            mask_save_path = os.path.join(output_mask_path, mask_filename)
            cv2.imwrite(mask_save_path, mask)
            self.get_logger().info(f"掩码图已保存：{mask_save_path}")
            # -------------------------- 新增部分结束 --------------------------

        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败：{e}")
        except Exception as e:
            self.get_logger().error(f"图像处理失败：{e}")

def main(args=None):
    rclpy.init(args=args)  # 保留args参数，兼容命令行传参
    node=HandDetectNode()
    
    # 补充：你原始代码遗漏了「订阅图片话题」的关键代码！
    # 必须添加这行，否则img_callback永远不会被触发
    node.img_sub = node.create_subscription(
        Image,
        "/camera/rgb/image_raw",  # 原始图片话题（根据你的实际话题修改）
        node.img_callback,
        10
    )
    
    rclpy.spin(node)
    # 销毁节点（释放资源）
    node.destroy_node()
    rclpy.shutdown()

# 程序入口
if __name__ == "__main__":
    main()