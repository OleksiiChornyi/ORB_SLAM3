# Prepare some required deps
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev
sudo apt-get install libcanberra-gtk-module

# Install Ros noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

# Install Mavros
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
cd ~/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Install Camera read node
cd ~/
mkdir -p camera_node/src
cd camera_node/src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
cd raspicam_node
git checkout 0.5.0
echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/30-ubiquity.list > /dev/null
rosdep update
cd ~/camera_node
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make
echo "source ~/camera_node/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
# This requires to reboot raspbrerry. Then it will start using a camera.
echo "start_x=1" | sudo tee -a /boot/firmware/config.txt >> /dev/null

# Install opencv
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.4.0

hpp_file_to_extend='./modules/videoio/src/cap_ffmpeg_impl.hpp'
echo "$(echo -en '#define AVFMT_RAWPICTURE 0x0020\n'; cat $hpp_file_to_extend)" > $hpp_file_to_extend
echo "$(echo -en '#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER\n'; cat $hpp_file_to_extend)" > $hpp_file_to_extend
echo "$(echo -en '#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)\n'; cat $hpp_file_to_extend)" > $hpp_file_to_extend

mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j 4
sudo make install

# Install Pangolin
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 4
sudo make install

# In case your rpi4 has 4GB memory, allocate extra 4GB swap:
# sudo dd if=/dev/zero of=/swapfile bs=1M count=4096 
# sudo chmod 0600 /swapfile 
# sudo mkswap /swapfile  
# sudo swapon /swapfile

# Then turn it off after installation:
# sudo swapoff -a

# Install and build ORB_SLAM3
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
sudo apt install ros-noetic-tf ros-noetic-image-transport ros-noetic-cv-bridge
sudo rosdep update

echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH::~/ORB_SLAM3/visual_stabilization/ros1/ORB_SMAL3" >> ~/.bashrc
source ~/.bashrc
cd ~/ORB_SLAM3
./build.sh
./visual_stabilization/build_ros1.sh
