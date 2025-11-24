FROM orsf/ros:humble-desktop-full

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
ENV SHELL=/bin/bash

# Switch from root to the new user
USER $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# Source ROS2 setup file
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

##############################
# Add ant custom setup below #
##############################
# Source the Colcon setup file
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

