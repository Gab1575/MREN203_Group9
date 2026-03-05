SMORES is proposing HERSHEY; a fully autonomous rover capable of completing this task. HERSHEY will be capable of navigating itself to the Three Forks Junction from any nearby Mars landing site and will automatically retrieve samples using camera based object detection and a fork-lift mechanism for sample collection and storage. While HERSHEY is designed for fully autonomous operation, it will incorporate robust Human Machine Interface (HMI) features to allow for monitoring and manual takeover if necessary.


Setting UP RVIZZ, SLAMTOOLBOX.

RVIZ:

Install: sudo apt install ros-humble-rviz2

Source: source /opt/ros/humble/setup.bash
Launch: ros2 run rviz2


SLAMTOOLBOX:
INSTALL: sudo apt install ros-humble-slam-toolbox

Source: source /opt/ros/humble/setup.bash
Launch: ros2 launch slam_toolbox online_async_launch.py


