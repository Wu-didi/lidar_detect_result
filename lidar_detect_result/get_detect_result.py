#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class Bbox:
    """用于存放每个检测框信息的简单类"""
    def __init__(self, x, y, z, w, l, h, theta, score, label):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.l = l
        self.h = h
        self.theta = theta
        self.score = score
        self.label = label

    def __str__(self):
        return (f"Bbox(x={self.x}, y={self.y}, z={self.z}, w={self.w}, "
                f"l={self.l}, h={self.h}, theta={self.theta}, "
                f"score={self.score}, label={self.label})")

class BboxSubscriber(Node):
    def __init__(self):
        super().__init__('bbox_subscriber')
        # 订阅和发布节点中对应的话题相同，这里是 'result_topic'
        self.subscription = self.create_subscription(
            String,
            'detection_results',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        回调函数：从订阅到的 String 中解析出 bounding box 信息。
        示例解析文本格式类似：
            检测到 2 个box:
              [0]: x=1.1, y=2.2, z=3.3, w=4.4, l=5.5, h=6.6, theta=0.7, score=0.95, label=car
              [1]: x=2.2, y=3.3, z=4.4, w=5.5, l=6.6, h=7.7, theta=1.1, score=0.90, label=truck
        """
        data_str = msg.data
        self.get_logger().info(f"收到原始消息:\n{data_str}")

        # 先解析出有多少个 box（可选，如果你需要此信息）
        first_line_pattern = r'检测到\s+(\d+)\s+个box'
        first_line_match = re.search(first_line_pattern, data_str)
        if not first_line_match:
            self.get_logger().warn("未能解析到有效的box数量，检查字符串格式。")
            return
        
        num_boxes = int(first_line_match.group(1))
        self.get_logger().info(f"解析到 box 数量: {num_boxes}")

        # 正则匹配每行内容：形如
        #   [i]: x=..., y=..., z=..., w=..., l=..., h=..., theta=..., score=..., label=...
        #
        # 注意：这里的正则假设浮点数里不会有空格等特殊情况，label 为单词字符。
        box_line_pattern = (
            r'\[(\d+)\]:\s*'
            r'x=([-\d.]+),\s*'
            r'y=([-\d.]+),\s*'
            r'z=([-\d.]+),\s*'
            r'w=([-\d.]+),\s*'
            r'l=([-\d.]+),\s*'
            r'h=([-\d.]+),\s*'
            r'theta=([-\d.]+),\s*'
            r'score=([-\d.]+),\s*'
            r'label=(\S+)'
        )

        # 用 findall 从整段文本中找出所有匹配
        matches = re.findall(box_line_pattern, data_str)
        if not matches:
            self.get_logger().warn("未能匹配到任何 bounding box 信息。")
            return

        bboxes = []
        for match in matches:
            # match 返回的是 (index, x, y, z, w, l, h, theta, score, label)，都还是字符串
            # 我们可以根据需要，把数值型的转换为 float 或 int
            idx, x_str, y_str, z_str, w_str, l_str, h_str, theta_str, score_str, label_str = match
            box = Bbox(
                float(x_str),
                float(y_str),
                float(z_str),
                float(w_str),
                float(l_str),
                float(h_str),
                float(theta_str),
                float(score_str),
                label_str
            )
            bboxes.append(box)

        # 此处 bboxes 就是解析好的所有检测框，可以做你需要的操作：打印、可视化、处理等
        for i, box in enumerate(bboxes):
            self.get_logger().info(f"第 {i} 个 Bbox: {box}")

        # 如果想验证数量，也可检查 bboxes 的长度和 num_boxes 是否一致
        if len(bboxes) != num_boxes:
            self.get_logger().warn(f"声明的 box 数量 {num_boxes} "
                                   f"与实际解析到的 {len(bboxes)} 个不一致。")

def main(args=None):
    rclpy.init(args=args)
    node = BboxSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
