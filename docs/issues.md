# Known issues and used fixes

## 1. Missing libcrypto.so.1.1 for license_checker

Ubuntu 24.04 has moved from OpenSSL 1.1 (libcrypto.so.1.1) to OpenSSL 3 (libcrypto.so.3). But the ./license_checker needs libcrypto.so.1.1 to run, resulting in a lib missing runtime error. This has been reported at https://github.com/graspnet/anygrasp_sdk/issues/136 and until issue is resolved, following fix has been applied to the dockerfile to install libcrypto.so.1.1. 

```bash
wget https://www.openssl.org/source/openssl-1.1.1o.tar.gz
tar -zxvf openssl-1.1.1o.tar.gz
cd openssl-1.1.1o
./config
make
make test
sudo make install
sudo find / -name libssl.so.1.1
sudo ln -s /usr/local/lib/libssl.so.1.1  /usr/lib/libssl.so.1.1
sudo find / -name libcrypto.so.1.1
sudo ln -s /usr/local/lib/libcrypto.so.1.1 /usr/lib/libcrypto.so.1.1
```

## 2. `ifconfig` is missing for license_checker

net_tools has been added via apt-get for the docker container

## 3. System-wide python package installation blocked

docker container uses following command to unblock system package installation.

```bash
python -m pip config set global.break-system-packages true
```

## 4. Feature ID not being fixed for docker

This devcontainer uses a docker network `bridge` and a fixed mac address to stablize the feature id.

## 5. Devcontainer not connecting to robot with bridge networking

If you are running inside the devcontainer with Docker bridge networking, set `reverse_ip` to the host machine's IP on the robot network so the robot can connect back to ports `50001`, `50003`, and `50004`. 

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

while in the ur10.launch.py, set the default value of `reverse_ip` to the host machine's IP on the robot network.

```python
DeclareLaunchArgument('reverse_ip', default_value=' <host_robot_network_ip>'),
```

Then launch the arm control node with:

```bash
source install/setup.bash
ros2 launch grasping_control ur10.launch.py
``` 