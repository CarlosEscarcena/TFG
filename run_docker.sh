docker run -it \
  --name ros2_camera \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  -v /usr/lib/libcamera:/usr/lib/libcamera \
  -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu \
  -v ~/ros2_ws:/root/ros2_ws \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros:humble
