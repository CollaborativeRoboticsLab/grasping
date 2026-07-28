# LD250 TM12X Connection with Devcontainer

If you are using the devcontainer and need to reach the physical TM12X controller from inside the container, make sure the container can access the robot network and pass the robot IP into the launch command.

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

The combined LD250 TM12X bringup still accepts `tm_robot_ip` and forwards it to `tm_driver`.

If you want a persistent default for the manipulator-only bringup, set the default value of `tm_robot_ip` in `ld250_tm12x_soft_two_fingers_moveit_config/launch/hardware_with_moveit.launch.py` to the robot controller IP.

```python
DeclareLaunchArgument('tm_robot_ip', default_value='<robot_controller_ip>'),
```

Then launch the combined manipulator stack with:

```bash
source install/setup.bash
ros2 launch ld250_tm12x_soft_two_fingers_moveit_config hardware_with_moveit.launch.py tm_robot_ip:=<robot_controller_ip>
```

If you are launching the full mobile manipulator platform instead, the top-level `moma_ros` launch forwards the same argument:

```bash
source install/setup.bash
ros2 launch moma_ros ld250_tm12x.launch.py tm_robot_ip:=<robot_controller_ip>
```