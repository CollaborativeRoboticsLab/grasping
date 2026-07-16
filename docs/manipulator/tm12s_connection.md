# TM12S connection with devcontainer

If you are using the devcontainer and need to reach the physical TM12S controller from inside the container, make sure the container can access the robot network and pass the robot IP into the launch command.

In the `devcontainer.json`, add:

```json
  "runArgs": [
    "--network=bridge",
    "-p",
    "50001:50001",
    "-p",
    "50002:50002",
    "-p",
    "50003:50003",
    "-p",
    "50004:50004"
  ],
```

The TM soft-two-finger bringup accepts `tm_robot_ip` and forwards it to `tm_driver`.

If you want a persistent default, set the default value of `tm_robot_ip` in `tm12s_soft_two_fingers_moveit_config/launch/hardware_with_moveit.launch.py` to the robot controller IP.

```python
DeclareLaunchArgument('tm_robot_ip', default_value='<robot_controller_ip>'),
```

Then launch the arm control node with:

```bash
source install/setup.bash
ros2 launch grasping_control tm12s_soft_two_fingers.launch.py tm_robot_ip:=<robot_controller_ip>
```