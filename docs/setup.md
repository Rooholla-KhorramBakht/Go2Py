# System Setup

The GO2 EDU comes with an onboard Jetson Orin NX, a Hessai LX-16 LiDAR sensor, and an intel Realsense D435i camera. This setup procedure targets the onboard Jetson Orin with IP `192.168.123.18` and presents the instructions for installing the Go2Py. We use the Linux systemd infrastructure to automatically run the components of the Go2Py as the robot is turned on. Each service runs its particular docker image so the installation of these services involves two steps; first building the required images and then installing the service files that launch them. Before installing these services though, you would need to perform the following steps to get an internet connection on the robot and install docker on it. 

## Internet Sharing

In order to access the internet on the Jetson computer, we hook the robot to a host development computer with internet access and configure it to share its connection with the robot. To configure the host computer, the following steps should be taken:
### Host Computer
The following steps configure the host computer to share its internet with the robot.
#### Enable IP forwarding:

```bash
sudo systemctl -w net.ipv4.ip_forward=1
```
#### Configure the iptables:

```bash
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
```
Note that `wlan0` should be replaced with the actual name of the network interface over which the internet is provided to the host computer, and eth0 should be replaced with the name of the Ethernet interface connected to the robot and has an IP address in range `192.168.123.x`. 

#### Storing the Settings
Make the iptables rules persistent by installing the `iptables-persistent`:

```bash
sudo apt-get install iptables-persistent
sudo iptables-save > /etc/iptables/rules.v4
sudo ip6tables-save > /etc/iptables/rules.v6
```
### Robot
Now tell the computer on the robot to use the internet shared by the host computer. SSH into the robot's computer with IP address `192.168.123.18`, username `unitree`, and password `123`. Note that the host computer's IP range should have already been set to static mode with an IP in the `192.168.123.x` range where x is anything except IPs already used by the others (e.g. `.18`).

```bash
sudo ip route add default via <host computer IP address>
```

Finally, configure the DNS server by adding the following to the `/etc/resolv.conf` file:
```bash
nameserver 8.8.8.8
```
**Note:** Similar to the host computer, you can save this configuration using the `iptables-persistent` tool.

If everything has been successful, you should be able to access the internet on the robot. Run `ping 8.8.8.8` and `ping www.google.com` to verify this. 

## Installing the Docker
All the deployed services in Go2Py are based on docker so make sure that docker and Nvidia runtime is installed. For this, you can follow through the Isaac-ROS installation instructions [here](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_storage.html).  

## Installing the Go2Py

Go2Py has two main components. A set of services running on the robot and a Python package that can run anywhere and is used to communicate with the robot in Python. 

### Installing the Services
SSH into the robot and clone the Go2Py repository:
```bash
git clone https://github.com/Rooholla-KhorramBakht/Go2Py.git
cd Go2Py
```
Go2Py is comprised of the following main services:
- **go2py-bridge.service:** A service that runs the Go2py C++ bridge.
- **go2py-robot-description.service:** A service that subscribes to the `\go\joint_states` published by the bridge and publishes the robot description and TF2 messages for the sensors and links of the robot. Note that you should add the correct extrinsic parameters of the installed sensors into the robot's xacro file located [here](../deploy/ros2_nodes/go2_description/xacro/robot.xacro) before installing this service. 
- **go2py-hesai.service:** This service runs a customized Lidar driver for the robots that come with the Hesai xt16 Lidars. This customized driver publishes laser_scan messages in addition to the pointcloud data. If your robot does not come with this LiDAR, you don't need to install this service.

The installation for each service is comprised of two commands:
```bash
# For the bridge
make bridge
sudo make bridge_install
# For the robot description service
make robot_description
sudo make robot_description_install
# For the LiDAR driver
make hesai
sudo make hesai_install
```
The first command for each service makes the corresponding docker image and the second command, copies the service file that runs those images into the appropriate locations of the system and enables them as autorun services. You can check for the success of this installation by checking the output of the `systemctl status service_name.service` command where the `service_name` is replaced with the name of the service you want to check. 

### Installing the GoPy 

Finally, we need to install the Go2Py Python library on a computer located on the same network as the robot (it could be the onboard computer but can also be any other PCs hooked up to the robot's network). To do so, simply go to the root directory of the Go2Py repository and run:
```bash
pip install -e .
```
To check the installation, run the interface example [here](../examples/00-robot-interface.ipynb) to make sure you can read the state of the robot. In addition to local installation, you can also the provided docker support. To do so, run `make docker_start` in the root directory of the repository:
```bash
cd ~/Go2Py
make docker_start
```

With this, a docker container with all the required dependencies will be launched and the Go2Py repository will be mounted into `/workspaces/Go2Py` and installed in editable mode. Since our docker support is based on the Nvida Isaac-ROS images, you can also use the Iasaac-ROS nodes within this environment.  

**Note:** Our base docker image can be extended in a similar way to the Isaac-ROS images (explained [here](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html)). Simply append the `CONFIG_IMAGE_KEY="ros2_humble.go2py"` [here](../scripts/.isaac_ros_common-config) with the key to your custom Dockerfile name (e.g. `CONFIG_IMAGE_KEY="ros2_humble.go2py.custom"` for `Dockerfile.custom`) and place your docker file under `docker` directory [here](../docker) and append its name with the key you used (e.g. `Dockerfile.custom`). Your custom Dockerfile should start with:
```docker
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
```
