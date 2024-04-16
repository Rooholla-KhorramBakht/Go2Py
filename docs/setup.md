# System Setup

The GO2 EDU comes with an onbard Jetson Orin NX, a Hessai LX-16 LiDAR sensor, and an intel Realsense D435i camera. This setup procedure targets the onboard Jetson Orin with IP `192.168.123.18` and presents the installation proceudures of nodes for reading the sensors and robot states and publisheing them as ROS2 topics plus some basic configurations for setting up the autostart services and sharing internet. The architecture of the GO2 system is illustrated in the following image:
TODO: add image

### Internet Sharing

In order to access internet on the Jetson computer, we hook the robot to a host development computer with internet access and configure it to share its connection with the robot. To cofigure the host computer, the follwoing steps should be taken:
#### Host Computer
The following steps configures the host computer to share its intentrent with the robot.
##### Enable IP forwarding:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
```
##### Configure the iptables:

```bash
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
```
Note that `wlan0` should be replaced with the actual name of the network interface over which the internet is provided to the host computer, and eth0 should be replaced with the name of the Ethernet interface connected to the robot and having a local IP address in robot's network range. 

##### Storing the Settings
Make the iptables rules persistent by installing the `iptables-persistent`:

```bash
sudo apt-get install iptables-persistent
sudo iptables-save > /etc/iptables/rules.v4
sudo ip6tables-save > /etc/iptables/rules.v6
```
#### Robot
Now tell the computer on the robot to use the internet shared by the host computer. SSH into the robot's computer with IP address `192.168.123.18`, username `unitree` and password `123`. Note that the host computer's IP reange should have already been set to static mode with an IP in `192.168.123.*` range.

```bash
sudo ip route add default via <host computer IP address>
```

Finally, configure the DNS server by adding the following to the `/etc/resolv.conf` file:
```bash
nameserver 8.8.8.8
```
**Note:** Similarly to the host computer, you can make save this configuration using the `iptables-persistent` tool.

If everything has been successful, you should be able to access the internet on the robot. Run `ping www.google.com` to verify this. 

