#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage



class ZedBridge(Node):
    def __init__(self) -> None:
        super().__init__('zed_image_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/zed/zed_node/rgb/image_rect_color/compressed',
            self._callback,
            1,
        )
        self.img_pub = self.create_publisher(CompressedImage, '/droid/wrist_image_left/compressed', 10)
        self.timer = self.create_timer(0.0333, self.publish_image) # pub at ~36Hz but eventually hz drops to match sub rate of ~30Hz
        self.last_image = None
    
    def _callback(self, msg: CompressedImage) -> None:
        self.last_image = msg

    def publish_image(self) -> None:
        if self.last_image is not None:
            img_msg = self.last_image
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
