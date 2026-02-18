# Known issues and fixes

## 1. Missing libcrypto.so.1.1 for license_checker

Ubuntu 24.04 has moved from OpenSSL 1.1 (libcrypto.so.1.1) to OpenSSL 3 (libcrypto.so.3). But the ./license_checker needs libcrypto.so.1.1 to run, resulting in a lib missing runtime error. This has been reported at https://github.com/graspnet/anygrasp_sdk/issues/136 and until issue is resolved, uses the following fix has been applied to the dockerfile to install libcrypto.so.1.1. 

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

This devcontainer uses a **macvlan** network (`ur_macvlan`) so the container appears directly on the LAN.

This is useful for discovering and connecting to other ROS nodes and robot hardware.

To keep AnyGrasp license “feature ID” stable across rebuilds, the devcontainer pins a deterministic MAC address via `--mac-address=02:42:de:ad:be:ef`.

See `README.md` → “Creating docker network” for the host-side `docker network create ...` command.

## 5. Host cannot reach container when using macvlan

With macvlan, it is common that the **host cannot talk to the container** directly (host ↔ container) even though other machines on the LAN can.

If you need host ↔ container connectivity, create an additional macvlan interface on the host (example only; adapt subnet/interface/IP to your LAN):

```bash
sudo ip link add macvlan-shim link eth0 type macvlan mode bridge
sudo ip addr add 192.168.1.250/24 dev macvlan-shim
sudo ip link set macvlan-shim up
```

Then use the container's macvlan IP to communicate from the host.