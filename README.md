# spraybot_utils
## Pause Bagfiles
ROS2 Galactic does not support pausing bagfiles with spacebar. By running this script in a different window we can achieve the same functionality.
Copied from https://github.com/ros2/rosbag2/pull/729 

1. Run in a new terminal with 
`ros2 run spraybot_utils pause_bagfiles.py`
2. Press spacebar to toggle pause/resume.
