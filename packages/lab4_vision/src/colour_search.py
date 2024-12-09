#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tb3 import Tb3Move


class ColourSearch:
    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = ""  # fast, slow, or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(1)
        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))

        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # HSV thresholds for blue and cyan
        blue_lower = (115, 224, 100)
        blue_upper = (130, 255, 255)
        cyan_lower = (85, 200, 100)
        cyan_upper = (90, 250, 255)
        green_lower = (55, 100, 100)
        green_upper = (60, 255, 255)
        red_lower = (0, 210, 100)
        red_upper = (5, 255, 255)

        blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
        cyan_mask = cv2.inRange(hsv_img, cyan_lower, cyan_upper)
        green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
        red_mask = cv2.inRange(hsv_img, red_lower, red_upper)

        # Check for blue or cyan color
        self.process_mask(blue_mask, crop_img, color_name="Blue")
        self.process_mask(cyan_mask, crop_img, color_name="Cyan")
        self.process_mask(green_mask, crop_img, color_name="Green")
        self.process_mask(red_mask, crop_img, color_name="Red")

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def process_mask(self, mask, crop_img, color_name):
        m = cv2.moments(mask)
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            print(f"Detected {color_name} pillar. Blob size: {self.m00:.0f}. Position: {self.cy:.0f}")

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                if 460 <= self.cy <= 660:
                    if self.move_rate == "slow":
                        self.move_rate = "stop"
                        self.stop_counter = 30
                else:
                    self.move_rate = "slow"
            else:
                self.move_rate = "fast"

            if self.move_rate == "fast":
                print("MOVING FAST: Scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == "slow":
                print(f"MOVING SLOW: A blob of size {self.m00:.0f} detected at position {self.cy:.0f}.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == "stop" and self.stop_counter > 0:
                print(f"STOPPED: Blob centered at position {self.cy:.0f}. Countdown: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of size {self.m00:.0f} detected at position {self.cy:.0f}.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            self.robot_controller.publish()
            self.rate.sleep()


if __name__ == "__main__":
    node = ColourSearch()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass

