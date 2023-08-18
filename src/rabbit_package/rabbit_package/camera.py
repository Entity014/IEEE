import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
from rclpy import qos


class Camera(Node):
    def __init__(self):
        super().__init__("cam_node")
        self.sub = self.create_subscription(
            Image,
            "/depth/image_rect_raw",
            self.image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub
        self.min_distance = 0.15  # meter
        self.max_distance = 0.7  # meter
        self.cv_bridge = CvBridge()
        self.img = None

    def image_callback(self, msg):
        depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_image_clipped = np.clip(
            depth_image, self.min_distance * 1000, self.max_distance * 1000
        )
        normalized_depth = cv2.normalize(
            depth_image_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        color_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
        # self.get_logger().info(f"{depth_image}")
        cv2.imshow("frame", color_depth)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            # closing all open windows
            cv2.destroyAllWindows()
            exit()


def main(args=None):
    rclpy.init(args=args)
    cam = Camera()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
