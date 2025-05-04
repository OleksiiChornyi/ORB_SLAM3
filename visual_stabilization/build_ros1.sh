echo "Building ROS1 nodes"

cd visual_stabilization/ros1/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j 4
