# ROS 2

## Preparation

### Camera calibration

NOTE: This instruction is extension of https://github.com/UbiquityRobotics/raspicam_node?tab=readme-ov-file#calibration

1. Start left and right camra as described in `Start All` second step below.

2. Start calibration tool with you grid params e.g.

```
size="7x7"    # How many square your grid has? and substracted 1. (e.g. I had 8x8 grid)
square="0.02" # How big your square in meters? (e.g. I had 2cm square edge length)
```

```
ros2 run camera_calibration cameracalibrator \
  --size 7x7 \
  --square 0.02 \
  --no-service-check \
  --approximate 0.1 \
  --ros-args \
  --remap right:=/stereo/right/image_raw \
  --remap left:=/stereo/left/image_raw \
  --remap right_camera:=/stereo/right/camera_info \
  --remap left_camera:=/stereo/left/camera_info \
  --remap stereo:=true
```

3. Now move you grid to different poses in camera. Best follow advises from: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

4. Save you calibration results and extract `left.yaml` and `right.yaml` files from calibration zip to `/home/drones/ORB_SLAM3/visual_stabilization/calibration` folder.

5. Create SLAM calibration files based this following [this guide](./CreateCalibration.md)

### IMU calibration

1. Record only IMU for at least 4 hourse in stale possition. Start Mavros and run:

```
ros2 bag record -o /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise_bag --topics /mavros/imu/data_raw
```

```
rosbags-convert --src /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise_bag --dst /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise.bag
```

2. TODO: start docker

3. Calibration only IMU using [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)

```
rosrun allan_variance_ros cookbag.py --input /data/imu_noise.bag --output cooked.bag
rosrun allan_variance_ros allan_variance /data /data/allan.yaml
rosrun allan_variance_ros analysis.py --data /data/allan_variance.csv
```

4. Now you have your [kalibr_imu.yaml](./visual_stabilization/calibration/kalibr_imu.yaml)

### IMU + camera calibration

1. Record april grid using drone with IMU and left camera running as in [this youtube guidance](https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY&ab_channel=SimpleKernel).

```
ros2 bag record -o /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_bag --topics /stereo/left/image_raw /mavros/imu/data_raw
```

```
rosbags-convert --src /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_bag --dst /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_calib.bag
```

3. To get imu to camera calibration data:

```
FOLDER=/home/drones/ORB_SLAM3/visual_stabilization/calibration/
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr

source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/april_calib.bag \
    --target /data/april_6x6_A4.yaml \
    --models pinhole-radtan \
    --topics /stereo/left/image_raw

rosrun kalibr kalibr_calibrate_imu_camera \
    --bag /data/april_calib.bag \
    --target /data/april_6x6_A4.yaml \
    --imu /data/kalibr_imu.yaml \
    --cam /data/april_calib-camchain.yaml
```

## Start All

1. Start mavros

- sudo chmod 666 /dev/ttyACM0
- ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
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

```
CAMERA_TOPIC_NAME_LEFT="/stereo/left/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/stereo/right/image_raw/compressed"
IMU_TOPIC_NAME="/mavros/imu/data_raw"
VOCABLUARY="/home/drones/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="/home/drones/ORB_SLAM3/visual_stabilization/calibration/rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $IMU_TOPIC_NAME $VOCABLUARY $CAMERA_CALIBRATION
```
