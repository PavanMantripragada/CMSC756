FROM shadowrobot/dexterous-hand:melodic-night-build
LABEL Description="This is updated to use OpenGL with nvidia-docker2" Vendor="Shadow Robot" Version="1.0"

RUN /bin/bash -c "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    USER=user \
    LANG=en_US.UTF-8 \
    HOME=/home/user \
    XDG_RUNTIME_DIR=/run/user/1000 \
    TZ=America/New_York

# Docker GPU access
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    software-properties-common \
    build-essential \
    g++ \
    git \
    ca-certificates \
    make \
    automake \
    autoconf \
    libtool \
    pkg-config \
    python \
    libxext-dev \
    libx11-dev \
    doxygen \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/*
# Setup tmux config
ADD https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.conf /home/$USER/.tmux.conf

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
# --------------------------------------------- #

# Set datetime and timezone correctly
# Remove duplicate sources
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    sudo rm /etc/apt/sources.list.d/ros1-latest.list

# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    python-catkin-tools \
    ros-melodic-usb-cam \
    ros-melodic-aruco-detect \
    ros-melodic-openni2-launch \
    ros-melodic-moveit-visual-tools

# Python3 for ROS and dependencies for shadowevents
RUN sudo apt-get update && \
    sudo apt-get install -y python3-pip python3-yaml && \
    sudo -H python3 -m pip install rospkg catkin_pkg matplotlib scipy numpy tqdm && \
    sudo apt-get install python3-tk

# pybind11
RUN sudo apt update && \
    sudo apt install -y --no-install-recommends --allow-unauthenticated python3.6-dev
RUN sudo -H python3 -m pip install pytest
RUN mkdir -p /home/$USER/apps && \
    cd /home/$USER/apps && \
    git clone https://github.com/pybind/pybind11.git && \
    cd pybind11 && \
    mkdir -p /home/$USER/apps/pybind11/build && \
    cd /home/$USER/apps/pybind11/build && \
    cmake -DPYBIND11_CPP_STANDARD=-std=c++11 -DPYTHON_EXECUTABLE=/usr/bin/python3.6 -DCMAKE_INSTALL_PREFIX=/usr/local -DPYBIND11_TEST=OFF -DPYBIND11_INSTALL=ON ../ && \
    make -j && \
    sudo make install

# Latest Eigen
RUN cd /home/$USER/apps && \
    git clone --depth 1 --branch 3.3.0 https://gitlab.com/libeigen/eigen.git && \
    mkdir -p /home/$USER/apps/eigen/build && \
    cd /home/$USER/apps/eigen/build && \
    cmake ../ && \
    make -j && \
    sudo make install

# cmake 3.16
ADD https://cmake.org/files/v3.16/cmake-3.16.9-Linux-x86_64.sh /opt/cmake-3.16.9-Linux-x86_64.sh
WORKDIR /opt/
RUN sudo chmod +x /opt/cmake-3.16.9-Linux-x86_64.sh && \
    bash -c "yes Y | sudo /opt/cmake-3.16.9-Linux-x86_64.sh" && \
    bash -c "sudo ln -s /opt/cmake-3.16.9-Linux-x86_64/bin/* /usr/local/bin"

# Startup stuff
WORKDIR /home/$USER/workspace
CMD bash -c echo /usr/local/lib/x86_64-linux-gnu | sudo tee /etc/ld.so.conf.d/glvnd.conf && sudo ldconfig && terminator -T 'Dexterous Hand Container'