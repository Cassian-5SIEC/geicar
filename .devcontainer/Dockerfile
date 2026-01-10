FROM osrf/ros:humble-desktop-full

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y ros-humble-navigation2 \
                        ros-humble-nav2-bringup \   
                        ros-humble-slam-toolbox \
                        ros-humble-robot-localization \
                        ros-humble-robot-state-publisher \
                        ros-humble-moveit \
                        ros-humble-ros-gz \
                        ros-humble-ros2-control \
                        ros-humble-ros2-controllers \
                        ros-humble-gz-ros2-control \
                        ros-humble-apriltag-ros \
                        ros-humble-foxglove-bridge \
                        ros-humble-rmw-cyclonedds-cpp
ENV SHELL=/bin/bash

# Switch from root to the new user
USER $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# Source ROS2 setup file
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_DOMAIN_ID=3" >> /home/$USERNAME/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

##############################
# Add ant custom setup below #
##############################
# Source the Colcon setup file
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

