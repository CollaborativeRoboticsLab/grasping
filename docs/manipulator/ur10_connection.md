# UR10 connection with devcontainer

If you are using the devcontainer with Docker bridge networking, set `reverse_ip` to the host machine's IP on the robot network so the robot can connect back to ports `50001`, `50003`, and `50004`. 

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

If you want a persistent default, set the default value of `reverse_ip` in `ur10_soft_two_fingers_moveit_config/launch/hardware_with_moveit.launch.py` to the host machine's IP on the robot network.

```python
DeclareLaunchArgument('reverse_ip', default_value='<host_computer_ip>'),
```

Then launch the arm control node with:

```bash
source install/setup.bash
ros2 launch grasping_control ur10_soft_two_fingers.launch.py reverse_ip:=<host_computer_ip>
```