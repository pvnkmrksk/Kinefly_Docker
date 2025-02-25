# Use Ubuntu 16.04 as the base image
FROM ubuntu:16.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kinetic

# Install necessary packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    sudo \
    wget \
    git \
    curl \
    python-setuptools \
    python-scipy \
    libdc1394-22-dev \
    intltool \
    gobject-introspection \
    python-opencv \
    xauth \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS repositories and install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y \
        ros-${ROS_DISTRO}-desktop-full \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        ros-${ROS_DISTRO}-driver-base \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Fix OpenCV issue
RUN mv /opt/ros/${ROS_DISTRO}/lib/python2.7/dist-packages/cv2.so /opt/ros/${ROS_DISTRO}/lib/python2.7/dist-packages/cv2.so.backup

# # Install Aravis
# WORKDIR /tmp
# RUN wget http://mirror.accum.se/pub/GNOME/sources/aravis/0.3/aravis-0.3.7.tar.xz \
#     && tar -xf aravis-0.3.7.tar.xz \
#     && cd aravis-0.3.7 \
#     && ./configure \
#     && make && make install

# Create catkin workspace
RUN mkdir -p ~/catkin/src
WORKDIR /root/catkin/src
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace"

# Clone necessary repositories
# RUN git clone https://github.com/florisvb/camera_aravis \
#     && cd camera_aravis \
#     && git checkout 497e415c5b0b9c20ac4179f8acc8ae9547799523 \
#     && cd .. \

RUN git clone https://github.com/psilentp/phidgets.git \
#     && git clone https://github.com/ssafarik/Kinefly \
    && git clone https://github.com/ssafarik/ledpanels \
    && git clone https://github.com/pvnkmrksk/Kinefly.git \
    && cd Kinefly\
    && git checkout 42f8d1a85ebbaf828fa6157f56257d1492d09d71 \
    && cd ..


# Install libphidget and PhidgetsPython
WORKDIR /tmp
RUN wget https://raw.githubusercontent.com/ungrinlab/monitor/master/libphidget.tar.gz \
    && tar -xzf libphidget.tar.gz \
    && cd libphidget-2.1.8.20140319 \
    && ./configure && make && make install \
    && wget https://raw.githubusercontent.com/ungrinlab/monitor/master/PhidgetsPython.zip \
    && unzip PhidgetsPython.zip \
    && cd PhidgetsPython \
    && python setup.py install

# # Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-uvc-camera\
    ros-kinetic-usb-cam


# Build the workspace
WORKDIR /root/catkin
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make  "

# Setup environment
RUN echo "source ~/catkin/devel/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash rosmake Kinefly"

# Install guvcview
RUN apt-get update && apt-get install -y \
    v4l-utils \
    guvcview \
    gedit


# Copy our modified launch files
COPY launch_files/rhag/_main.launch /root/catkin/src/Kinefly/launch/rhag/
COPY launch_files/rhag/camera_1394.launch /root/catkin/src/Kinefly/launch/rhag/




# Set up the entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Final setup steps
RUN echo "export RIG=rhag" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin/devel/setup.bash && rosmake Kinefly"

ENTRYPOINT ["/entrypoint.sh"]
