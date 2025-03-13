FROM osrf/ros:noetic-desktop-full

# add sourcing to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install dependencies
RUN apt update && apt install -y git
RUN apt install nano ros-noetic-tf2-tools evince -y
RUN apt install ros-noetic-moveit ros-noetic-franka-ros  -y
RUN apt install python3-pip python3-tk -y
RUN apt install gnome-terminal -y

# install python dependencies to run frankapy within the docker container
RUN pip3 install autolab_core 
RUN pip3 install --force-reinstall pillow==9.0.1 && pip3 install --force-reinstall scipy==1.8
RUN pip3 install numpy-quaternion numba && pip3 install --upgrade google-api-python-client 
RUN pip3 install --force-reinstall numpy==1.23.5

# Install realsense camera
ARG DEBIAN_FRONTEND=noninteractive
RUN apt install ros-noetic-realsense2-camera -y
RUN apt install ros-noetic-aruco-ros -y

# Install PyTorch
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# mount src folder from desktop to /home/ros_ws/src
COPY src/devel_packages /home/ros_ws/src/devel_packages

# clone git packages
RUN mkdir -p /home/ros_ws/src/git_packages 
RUN cd /home/ros_ws/src/git_packages && git clone --recursive https://github.com/iamlab-cmu/frankapy.git \
        && git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel \
        && git clone https://github.com/IFL-CAMP/easy_handeye

# add frankapy to python path
ENV PYTHONPATH "${PYTHONPATH}:/home/ros_ws/src/git_packages/frankapy"

# Install Depencencies via rosdep on src folder
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/ros_ws; rosdep install --from-paths src --ignore-src -r -y"

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/ros_ws; catkin_make"

# Add sourcing ros ws to bashrc
RUN echo "source /home/ros_ws/devel/setup.bash" >> ~/.bashrc

## Azure kinect installation  steps
# Update package lists
RUN apt-get update && apt-get upgrade -y

# Install necessary packages
RUN apt-get install -y curl openssh-server git vim terminator software-properties-common autoconf automake libtool make g++ unzip python3-distutils

# Install additional Python packages
RUN pip3 install catkin-tools empy

# Add ROS setup.bash to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install libsoundio1
RUN apt-get install -y libsoundio1

# Download Azure Kinect packages
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb \
    && wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb \
    && wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb

# Install Azure Kinect packages
RUN ACCEPT_EULA=y dpkg -i libk4a1.4* k4a-tools*

# Clean up downloaded packages
RUN rm libk4a1.4*
RUN rm k4a-tools*

# Clone Azure Kinect ROS Driver
RUN cd /home/ros_ws/src/git_packages && git clone --recursive https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b melodic
RUN cd /home/ros_ws/src/git_packages/Azure_Kinect_ROS_Driver && git checkout melodic
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/ros_ws; catkin_make"

# Add sourcing ros ws to bashrc
RUN echo "source /home/ros_ws/devel/setup.bash" >> ~/.bashrc

# set workdir as home/ros_ws
WORKDIR /home/ros_ws

CMD [ "bash" ]