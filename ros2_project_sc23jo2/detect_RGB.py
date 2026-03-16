import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal
from std_msgs.msg import Bool


class Detect_RGB(Node):

    def __init__(self):
        super().__init__('detect_rgb')

        self.bridge = CvBridge()

        # Subscribe to camera
        self.subscription = self.create_subscription( Image, '/camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blue_pub = self.create_publisher(Bool, '/blue_detected', 10)

        self.sensitivity = 10
        self.too_close = False

    def callback(self, data):

        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Green
        green_lower = np.array([60 - self.sensitivity, 100, 100])
        green_upper = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Blue
        blue_lower = np.array([120 - self.sensitivity, 100, 100])
        blue_upper = np.array([120 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # Red
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])

        red_lower2 = np.array([170, 100, 100])
        red_upper2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)

        red_mask = red_mask1 + red_mask2

        # Detect colours
        self.detect_colour(green_mask, image, "Green", (0,255,0))
        self.detect_colour(red_mask, image, "Red", (0,0,255))
        self.detect_blue(blue_mask, image)

        cv2.imshow("camera_feed", image)
        cv2.waitKey(3)


    def detect_colour(self, mask, image, name, draw_colour):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                (x,y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, draw_colour, 2)
                self.get_logger().info(f"{name} detected")
    

    def detect_blue(self, mask, image):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 100:
                (x,y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (255,0,0), 2)

                self.get_logger().info("Blue detected")
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    cv2.circle(image, (cx, cy), 5, (255,255,255), -1)
                    image_center = image.shape[1] / 2
                    twist = Twist()

                    if area > 300000: # Equates to 1 grid away
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Reached blue box")
                    else:
                        if cx < image_center - 80:
                            # turn left
                            twist.angular.z = 0.1
                        elif cx > image_center + 80:
                            # turn right
                            twist.angular.z = -0.1
                        else:
                            # go straight
                            twist.linear.x = 0.3
                            twist.angular.z = 0.0
                    self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = Detect_RGB()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()