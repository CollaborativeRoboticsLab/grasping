## Calibration of the UR10 robot arms

This document covers the calibration of the UR10 robot arms for use with the `grasping_arm_control` node. This process follows the guidelines in the official UR documentation.

Follow the following steps to calibrate the robot:

1. Setup the [robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup) 
2. Setup the [network connection](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html#network-setup)
3. Perform [Calibration](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html)

    ```bash
    ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=10.0.0.89 target_filename:="/home/ubuntu/colcon_ws/src/ur_grasping/grasping_arm_control/config/ur_kinematics.yaml"
    ```