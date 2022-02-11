# turtlebot4_robot
Robot packages for the Turtlebot4.

# Initial Setup

When you begin to use the Turtlebot4, you will first have to take some steps to set it up.

## RPI4 WiFi Setup

- Power on the robot by placing it on its dock.

- The Raspberry Pi will boot after a few moments. 

- On the first boot, the Raspberry Pi will enter AP mode which will allow you to connect to it over WiFi.

- On a PC, connect to the `Turtlebot4` WiFi network. The password is also `Turtlebot4`.

- Once connected, you can SSH into the Raspberry Pi to configure its WiFi.

```bash
ssh ubuntu@10.42.0.1
```
- The default password is `turtlebot4`

- In the home folder there will be a script called `wifi.sh` which can be used to configure the Raspberry Pi's WiFi:
```bash
bash ~/wifi.sh -s "YOUR_WIFI_SSID" -p "YOUR_WIFI_PASSWORD" -c YOUR_REGULATORY_DOMAIN && sudo reboot
```
Note: The Regulatory domain is based on the country you live in. For a full list, click [here](https://www.arubanetworks.com/techdocs/InstantWenger_Mobile/Advanced/Content/Instant%20User%20Guide%20-%20volumes/Country_Codes_List.htm#regulatory_domain_3737302751_1017918).

- Your Raspberry Pi will reboot and connect to your WiFi network.
- On your PC, run `ros2 topic list` to ensure that the Turtlebot4 is publishing its topics.
- Run `ros2 topic echo /ip` to read the new IP of the Raspberry Pi. On the Turtlebot4 Standard this will also be displayed on the screen.
- You can now SSH into the Turtlebot4's Raspberry Pi at the new IP and begin using it.
```bash
ssh ubuntu@xxx.xxx.xxx.xxx
```

## Create3 WiFi Setup

- Press both Create3 button 1 and 2 simultaneously until light ring turns blue
- The Create3 is now in AP mode. Connect to its WiFi 'Create-XXXX'
- In a browser go to 192.168.10.1
- Click connect and enter your WiFi ssid and password
- Wait for it to connect to WiFi
- On your PC, run `ros2 topic list` to ensure that the Create3 is publishing its topics.

# Running Turtlebot4 manually

By default the Turtlebot4 will be running its node, diagnostics, and sensors in a service called `turtlebot4`. You can disable this by calling `sudo systemctl stop turtlebot4`. You can then run the individual components manually. To re-enable the service, call `sudo systemctl start turtlebot4`.

## Node

### Turtlebot4 Standard
`ros2 launch turtlebot4_bringup robot.launch.py model:=standard`

### Turtlebot4 Lite
`ros2 launch turtlebot4_bringup robot.launch.py model:=lite`

## RPLIDAR
`ros2 launch turtlebot4_bringup rplidar.launch.py`

## OAKD
`ros2 launch turtlebot4_bringup oakd.launch.py`

## SLAM
`ros2 launch turtlebot4_navigation slam.launch.py`

## Nav2
`ros2 launch turtlebot4_navigation nav2.launch.py`

## Diagnostics
`ros2 launch turtlebot4_diagnostics diagnostics.launch.py`

## Description
`ros2 launch turtlebot4_description robot_description.launch.py`

