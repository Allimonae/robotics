#!/usr/bin/env python3 

import rospy 
import cv2 
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image 
from tb3 import Tb3Move 

class LineFollower(object):  
    def __init__(self): 
        node_name = "line_follower" 
        rospy.init_node(node_name, anonymous=True) 
        self.rate = rospy.Rate(5) 

        self.cvbridge_interface = CvBridge() 
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb) 
        self.robot_controller = Tb3Move() 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdown_ops) 

    def shutdown_ops(self): 
        self.robot_controller.stop() 
        cv2.destroyAllWindows() 
        self.ctrl_c = True 

    def camera_cb(self, img_data): 
        try: 
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8") 
        except CvBridgeError as e: 
            print(e) 

        height, width, _ = cv_img.shape  
        crop_height = int(height * 0.3)  
        crop_y0 = height - crop_height
        cropped_img = cv_img[crop_y0:height, :]
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        
        lower_purple1 = (0, 120, 100)
        upper_purple1 = (25, 130, 255)
        lower_purple2 = (145, 124, 100)
        upper_purple2 = (180, 255, 255) 
        purple_mask1 = cv2.inRange(hsv_img, lower_purple1, upper_purple1)
        purple_mask2 = cv2.inRange(hsv_img, lower_purple2, upper_purple2)
        purple_mask = cv2.bitwise_or(purple_mask1, purple_mask2) 
        
        res = cv2.bitwise_and(cropped_img, cropped_img, mask=purple_mask) 
        #cv2.imshow("Purple Mask", purple_mask)
        cv2.waitKey(1)
        
        lower_red1 = (0, 200, 100)
        upper_red1 = (2, 255, 255)
        lower_red2 = (179, 200, 100)
        upper_red2 = (180, 255, 255)
        red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_not_purple_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(purple_mask))
        
        #cv2.imshow("Red Mask", red_mask)
        cv2.waitKey(1)
        #cv2.imshow("Red not purpule Mask", red_not_purple_mask)
        cv2.waitKey(1)
        
        if cv2.countNonZero(red_mask) > 0:
            rospy.loginfo("Red square detected! Stopping the robot.")
            self.robot_controller.set_move_cmd(0.0, 0.0)  # Stop the robot
            self.robot_controller.publish()
            return

        m = cv2.moments(purple_mask) 
        if m['m00'] == 0:
        	rospy.logwarn("Line not detected. Searching...")
        	self.robot_controller.set_move_cmd(0.0, 0.3)
        	self.robot_controller.publish()
        	return
        	
        cy = m['m10'] / m['m00'] 
        cz = m['m01'] / (m['m00'] + 1e-5) 
        
        # draw centroid
        cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2) 
        cv2.imshow("filtered image", res) 
        cv2.waitKey(1) 

	# calculate error and limit angular velocity
        y_error = cy - (width / 2) 
        
        rospy.loginfo(f"Centroid X: {cy}, Y-Error: {y_error}")
		
        kp = 0.001 
        fwd_vel = 0.5
        min_ang_vel = -1.0
        max_ang_vel = 1.0
        ang_vel = -kp * y_error
        threshold = 10
        
        if abs(y_error) < threshold:
        	rospy.loginfo("Line is centered, no turn needed")
        	self.robot_controller.set_move_cmd(fwd_vel, 0.0)
        	self.robot_controller.publish()
        	return 
        	
        print(f"Y-error = {y_error:.3f} pixels, ang_vel = {ang_vel:.3f} rad/s")   
        self.robot_controller.set_move_cmd(fwd_vel, ang_vel) 
        self.robot_controller.publish()  

    def main(self): 
        while not self.ctrl_c: 
            self.rate.sleep() 

if __name__ == '__main__': 
    lf_instance = LineFollower() 
    try: 
        lf_instance.main() 
    except rospy.ROSInterruptException: 
        pass
