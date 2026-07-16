# UR10 connection with devcontainer

If you are using the devcontainer with Docker bridge networking, set `reverse_ip` to the host machine's IP on the robot network so the robot can connect back to ports `50001`, `50003`, and `50004`. 

In the devccontainer.json add,

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

while in the `grasping_control/ur10.launch.py`, set the default value of `reverse_ip` to the host machine's IP on the robot network.

```python
DeclareLaunchArgument('reverse_ip', default_value=' <host_robot_network_ip>'),
```

Then launch the arm control node with:

```bash
source install/setup.bash
ros2 launch grasping_control ur10.launch.py