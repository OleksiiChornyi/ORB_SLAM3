echo "Building ROS2 nodes"

cd visual_stabilization/ros2
mkdir build
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
