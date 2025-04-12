
1. Connect to flight controller using MavRos to get all topics

Laptop to drone:

- sudo chmod 666 /dev/ttyACM0
- roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 mavlink_version:=2.0
- rosservice call /mavros/set_stream_rate 0 10 1

Rasberi to drone:

- roslaunch mavros apm.launch fcu_url:=/dev/serial0 navlink_version:=2.0 

2. Start image node

- roslaunch raspicam_node camerav2_1280x960.launch
