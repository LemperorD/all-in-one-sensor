cd ~/all-in-one-sensor
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DROS_EDITION="ROS2" -DDISTRO_ROS="humble"