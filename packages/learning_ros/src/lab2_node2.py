#!/usr/bin/env python3

import rospy

def set_distance():
    units = ["meters", "feet", "smoots"]
    
    current_index = 0

    while not rospy.is_shutdown():

        rospy.set_param('distance_unit', units[current_index])
        rospy.loginfo(f"Set distance_unit to {units[current_index]}")

        current_index = (current_index + 1) % len(units)
        rospy.sleep(5)

if __name__ == '__main__':
    # Start the ParamSetter node
    rospy.init_node('param_setter')
    try:
        set_distance()
    except rospy.ROSInterruptException:
        pass

