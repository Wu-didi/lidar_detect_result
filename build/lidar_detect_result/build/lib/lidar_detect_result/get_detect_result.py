import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResultSubscriber(Node):
    def __init__(self):
        super().__init__('result_subscriber')
        # 订阅名需要和发布方保持一致
        self.subscription_ = self.create_subscription(
            String,           # 消息类型
            'detection_results',  # 话题名称，要和发布方一致
            self.topic_callback,
            10                # 队列大小
        )
        self.subscription_  # prevent unused variable warning

    def topic_callback(self, msg):
        # 在回调函数里打印结果
        print("call back")
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ResultSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
