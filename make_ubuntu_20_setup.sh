sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev
sudo apt-get install libcanberra-gtk-module

cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.2.0

sed -i '#define AVFMT_RAWPICTURE 0x0020' | ./modules/videoio/src/cap_ffmpeg_impl.hpp
sed -i '#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER' | ./modules/videoio/src/cap_ffmpeg_impl.hpp
sed -i '#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)' | ./modules/videoio/src/cap_ffmpeg_impl.hpp

mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j 3
sudo make install
cd ..
mv opencv opencv4


cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 3 
sudo make install


sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH::~/ORB_SLAM3/Examples_old/ROS/ORB_SLAM3::~/ORB_SLAM3" >> ~/.bashrc
cd ~/ORB_SLAM3
./build.sh
./build_ros.sh
