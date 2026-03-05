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
**
Configure RViz to View the Map:**

With sensors and slam_toolbox running, open RViz:

    Set the Fixed Frame: Go to the "Displays" panel on the left. Expand Global Options and change the Fixed Frame to map.

    Add the Map Display: * Click the Add button at the bottom left.

        Select Map from the list and click OK.

        Expand the new Map display in the left panel.

        Find the Topic field and set it to /map. You should immediately see the 2D occupancy grid (the map) appear in the main window.

    Add Robot Model: Click Add -> RobotModel so you can see your robot's physical position on the map.

    Add TF (Optional but recommended): Click Add -> TF. This shows the coordinate frames. You will see slam_toolbox dynamically updating the transform between the map frame and the odom frame as the robot moves.
