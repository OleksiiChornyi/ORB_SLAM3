# ROS 2

## Preparation

1. Follow [full calibration guidance](./calibration/README.md) required for proper work of SLAM.
2. To communicate between your PC WSL and Raspberi Pi you need to follow some [network adjustments steps](./window_network.md).

## Start All

1. Start mavros

- sudo chmod 666 /dev/ttyACM0
- ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 fcu_protocol:="v2.0"
- ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"

Note: `ttyACM0` for USB connection

2. Start left and right camera with 30 fps

left
```
ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p camera_info_url:=file:///home/drones/ORB_SLAM3/visual_stabilization/calibration/left.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/stereo/left/image_raw \
  -r /camera/camera_info:=/stereo/left/camera_info \
  -r /camera/image_raw/compressed:=/stereo/left/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

right
```
ros2 run camera_ros camera_node --ros-args \
  -p camera:=1 \
  -p camera_info_url:=file:///home/drones/ORB_SLAM3/visual_stabilization/calibration/right.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/stereo/right/image_raw \
  -r /camera/camera_info:=/stereo/right/camera_info \
  -r /camera/image_raw/compressed:=/stereo/right/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

3. Start SLAM

Without IMU:
```
CAMERA_TOPIC_NAME_LEFT="/stereo/left/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/stereo/right/image_raw/compressed"
IMU_TOPIC_NAME="/mavros/imu/data_raw"
VOCABLUARY="/home/drones/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="/home/drones/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $VOCABLUARY $CAMERA_CALIBRATION
```

OR with IMU:
```
CAMERA_TOPIC_NAME_LEFT="/stereo/left/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/stereo/right/image_raw/compressed"
IMU_TOPIC_NAME="/mavros/imu/data_raw"
VOCABLUARY="/home/drones/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="/home/drones/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $IMU_TOPIC_NAME $VOCABLUARY $CAMERA_CALIBRATION
```
