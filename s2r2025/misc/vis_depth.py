#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DepthVisualizer(Node):
    running = True
    def __init__(self):
        super().__init__('depth_visualizer')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建订阅者
        self.rgb_sub = self.create_subscription(
            Image, 
            '/head_camera/color/image_raw', 
            self.rgb_callback, 
            qos
        )
        self.depth_sub = self.create_subscription(
            Image, 
            '/head_camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            qos
        )
        
        # 初始化图像变量
        self.rgb_image = None
        self.depth_image = None
        self.display_image = None
        self.split_position = 0.5  # 初始分割位置（窗口宽度的比例）
        self.window_name = "RGB-Depth Visualization"
        self.mouse_x = 0
        self.mouse_y = 0
        self.depth_value = 0
        
        # 创建窗口和鼠标回调
        cv2.namedWindow(self.window_name, cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # 创建定时器，定期更新显示
        self.timer = self.create_timer(1./24., self.update_display)
        
        self.get_logger().info('深度可视化节点已启动')
    
    def rgb_callback(self, msg):
        """RGB图像回调函数"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'RGB图像转换错误: {str(e)}')
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'深度图像转换错误: {str(e)}')
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        # 更新鼠标位置
        self.mouse_x = x
        self.mouse_y = y
        
        # 获取深度值
        if self.depth_image is not None and self.display_image is not None:
            # 计算在深度图像中的位置
            depth_x = int(x * (self.depth_image.shape[1] / self.display_image.shape[1]))
            depth_y = int(y * (self.depth_image.shape[0] / self.display_image.shape[0]))
            
            # 确保坐标在有效范围内
            depth_x = min(max(0, depth_x), self.depth_image.shape[1] - 1)
            depth_y = min(max(0, depth_y), self.depth_image.shape[0] - 1)
            
            # 获取深度值（单位：毫米）
            self.depth_value = self.depth_image[depth_y, depth_x]
        
        # 检查鼠标左键是否按下（包括按下和拖动）
        if flags & cv2.EVENT_FLAG_LBUTTON:
            # 左键按下时改变分割位置
            if self.display_image is not None:
                self.split_position = x / self.display_image.shape[1]
    
    def update_display(self):
        """更新显示"""
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # 确保两个图像尺寸一致
        if self.rgb_image.shape[:2] != self.depth_image.shape[:2]:
            rclpy.logging.get_warning_logger('depth_visualizer').warning('深度图像尺寸不匹配，已调整大小, depth_image.shape: %s, rgb_image.shape: %s' % (self.depth_image.shape, self.rgb_image.shape))
            self.depth_image = cv2.resize(self.depth_image, (self.rgb_image.shape[1], self.rgb_image.shape[0]))
        
        # 将深度图像转换为彩色图像以便可视化
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(self.depth_image, 0, 3000.), alpha=0.255/3.), cv2.COLORMAP_JET)
        
        # 创建显示图像
        h, w = self.rgb_image.shape[:2]
        split_x = int(w * self.split_position)
        
        # 创建组合图像
        self.display_image = np.zeros((h, w, 3), dtype=np.uint8)
        self.display_image[:, :split_x] = self.rgb_image[:, :split_x]
        self.display_image[:, split_x:] = depth_colormap[:, split_x:]

        # 添加深度值信息
        depth_text = f"depth: {self.depth_value} mm"
        cv2.putText(self.display_image, depth_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 显示图像
        cv2.imshow(self.window_name, self.display_image)
        key = cv2.waitKey(1)

        # ESC键退出
        if key == 27:
            self.running = False
            self.get_logger().info('退出深度可视化节点')
            raise KeyboardInterrupt
        # 'r'键重置分割位置
        elif key == ord('r'):
            self.split_position = 0.5

def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()