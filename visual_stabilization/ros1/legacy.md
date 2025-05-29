# ROS1

1. Connect to flight controller using MavRos to get all topics

Laptop to drone:

- sudo chmod 666 /dev/ttyACM0
- roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 mavlink_version:=2.0
- rosservice call /mavros/set_stream_rate 0 10 1

Check available tty connections:

- ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null


2. Start image node

- roslaunch raspicam_node camerav2_1280x960.launch

3. Start SLAM

```
CAMERA_TOPIC_NAME="/raspicam_node/image/compressed"
IMAGE_FORMAT="compressed_jpeg"
VOCABLUARY="/home/drones/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="/home/drones/ORB_SLAM3/rpi4_calibration.yaml"

rosrun ORB_SLAM3 Stabilization $CAMERA_TOPIC_NAME $IMAGE_FORMAT $VOCABLUARY $CAMERA_CALIBRATION
```

## Camera Calibration

NOTE: This instruction is extension of https://github.com/UbiquityRobotics/raspicam_node?tab=readme-ov-file#calibration

1. You need another local machine with GUI support and ROS configured.
2. Your raspberry and stationary machine should be connected over a network. Verify it by ssh communication.
3. In raspberry pi machine:
```
rpi_ip="192.168.88.213" # Replace with your raspberry ip

echo "export ROS_MASTER_URI=http://${rpi_ip}:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=${rpi_ip}" >> ~/.bashrc

source ~/.bashrc
```
4. In you stationary machine:
```
rpi_ip="192.168.88.213"        # Replace with your raspberry ip
stationary_ip="172.22.194.119" # Replace with your stationary machine ip

echo "export ROS_MASTER_URI=http://${rpi_ip}:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=${stationary_ip}" >> ~/.bashrc

source ~/.bashrc
```

5. Run in raspberry machine: `roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true` to start getting camera images

6. Run in you stationary machine to read raspberry images and calibrate image

```
grid_size="7x7"    # How many square your grid has? and substracted 1. (e.g. I had 8x8 grid)
square_size="0.02" # How big your square in meters? (e.g. I had 2cm square edge length)

rosrun camera_calibration cameracalibrator.py --size ${grid_size} --square ${square_size} image:=/raspicam_node/image camera:=/raspicam_node
```

7. Now move you grid to different poses in camera. Best follow advises from: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
