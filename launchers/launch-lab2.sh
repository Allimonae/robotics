#!/bin/bash

LAB_PACKAGE="learning_ros"
LAB_LAUNCH="launch_file2.launch"

source ~/catkin_ws/devel/setup.bash

# initialize launch file
# dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
roslaunch $LAB_PACKAGE $LAB_LAUNCH

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
