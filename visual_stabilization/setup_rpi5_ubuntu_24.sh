# Install ROS2
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

sudo apt install python3-rosdep
sudo rosdep init

# Install mavros
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
cd ~/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Install camera images reader
sudo apt update
sudo apt install -y git python3-pip python3-jinja2 libboost-dev openssl
sudo apt install -y libgnutls28-dev libtiff-dev pybind11-dev meson cmake
sudo apt install -y python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev

git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
sudo ninja -C build install

mkdir -p ~/rpi_camera/src
cd ~/rpi_camera/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/rpi_camera
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera

colcon build --event-handlers=console_direct+
echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/libcamera.conf
sudo chmod -R 777 /dev/
source ~/rpi_camera/install/setup.bash
echo "source ~/rpi_camera/install/setup.bash" >> ~/.bashrc

# Install exta deps
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libxvidcore-dev libx264-dev
sudo apt-get install libatlas-base-dev gfortran python3-dev libeigen3-dev libboost-all-dev libsuitesparse-dev
sudo apt install libegl-dev libgl1-mesa-dev libopengl-dev libepoxy-dev python3-pip colcon
python3.12 -m pip install --user wheel

# Opencv
sudo dpkg --remove --force-all python3-catkin-pkg
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro
sudo apt --fix-broken install
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-rosdistro-modules
sudo apt install libopencv-dev python3-opencv ros-jazzy-cv-bridge

# Install Pangolin
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j2
sudo make install

# Build ORB_SLAM3
cd ~/ORB_SLAM3

mv CMakeLists_ubuntu_24.txt CMakeLists.txt
./build.sh
./visual_stabilization/build_ros2.sh
source ~/ORB_SLAM3/visual_stabilization/ros2/install/setup.bash
echo "source ~/ORB_SLAM3/visual_stabilization/ros2/install/setup.bash" >> ~/.bashrc
