#!/usr/bin/env python3

import rospy
from turtlesim_helper.msg import UnitsLabelled

class ConversionNode:
    def __init__(self):
        rospy.init_node('convert_distance')
        
        # Initialize
        # self.prev_x = None
        # self.prev_y = None
        
        # Publisher
        self.pub = rospy.Publisher('converted_distance', UnitsLabelled, queue_size=10)
        
        # Subscribe to distance_traveled
        rospy.Subscriber('distance_traveled', UnitsLabelled, self.convert_distance_callback)
        
        # time to set parameter?
        rospy.sleep(1)


    def convert_distance_callback(self, msg):
        unit = rospy.get_param('distance_unit', "smoots")
        valid_units = ["meters", "feet", "smoots"]
        
        if unit not in valid_units:
            rospy.logwarn(f"Invalid unit '{unit}' received. Defaulting to 'meters'.")
            unit = "meters"
            
        rospy.loginfo(f"Received message: {msg.value} {msg.units}")
        output_msg = UnitsLabelled()

        if unit == "meters":
            output_msg.value = msg.value
            output_msg.units = "meters"
        
        elif unit == "feet":
            output_msg.value = msg.value * 3.28084
            output_msg.units = "feet"
        
        elif unit == "smoots":
            output_msg.value = msg.value / 1.7018
            output_msg.units = "smoots"
        
        # Publish the converted distance
        rospy.loginfo(f"Publishing: {output_msg.value} {output_msg.units}")
        self.pub.publish(output_msg)


if __name__ == '__main__':
    try:
        # Start the ConvertDistance node
        convert_distance_node = ConversionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
