FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/cavers_ws/src

RUN apt-get update && \
  apt-get install -y \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-realsense2-camera-msgs \
  ros-humble-velodyne-msgs 

COPY thermal_camera/ .

RUN cd .. && /bin/bash -c 'source /opt/ros/humble/setup.bash; colcon build'

WORKDIR /root

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
  echo 'source /root/cavers_ws/install/local_setup.bash' >> ~/.bashrc

COPY ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]

CMD ["/bin/bash"]