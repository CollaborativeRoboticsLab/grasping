# TF Frames of the system

Following are the color conventions used with Rviz2

- Red - X axis
- Green - Y axis
- Blue - Z axis
- Following transformations are ["x", "y", "z", "qx", "qy", "qz", "qw"]

## tool0 

This is last tf of the ur10 robot. This shows Z axis outwards, X axis downwards.

![Tool0 tf frame](../images/tool0_tf.png)

## gripper_mount

This is the outer surface of the camera mount and is a child of tool0.

tool0 -> gripper_mount transformation is, [0, 0, 0.008, 0, 0, 0, 0]