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


# Create directory for Kinefly bridge scripts
RUN mkdir -p /opt/Kinefly_docker

# Copy essential scripts and files to the container
COPY ros_zmq_bridge.py /opt/Kinefly_docker/
COPY test_zmq_client.py /opt/Kinefly_docker/
COPY test_camera.sh /opt/Kinefly_docker/
COPY requirements.txt /opt/Kinefly_docker/
COPY start-kinefly-all.sh /opt/Kinefly_docker/
RUN chmod +x /opt/Kinefly_docker/start-kinefly-all.sh /opt/Kinefly_docker/test_camera.sh

# Install Python dependencies for ZMQ bridge
RUN apt-get update && apt-get install -y \
    python-pip \
    python3-pip \
    && pip install "click==6.7" "pyzmq==17.1.2" \
    && pip3 install "click==7.0" "pyzmq==18.1.0" \
    && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Final setup steps
RUN echo "export RIG=rhag" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin/devel/setup.bash && rosmake Kinefly"

# Enhanced environment setup - Add all necessary environment variables and aliases
RUN echo "" >> ~/.bashrc \
    && echo "# === Kinefly Environment Setup ===" >> ~/.bashrc \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin/devel/setup.bash" >> ~/.bashrc \
    && echo "export RIG=rhag" >> ~/.bashrc \
    && echo "export PYTHONPATH=/root/catkin/src/Kinefly/src:\$PYTHONPATH" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# === Simple Aliases ===" >> ~/.bashrc \
    && echo "alias kinefly='/opt/Kinefly_docker/start-kinefly-all.sh'" >> ~/.bashrc \
    && echo "alias status='rostopic list | grep -E \"(kinefly|camera)\"'" >> ~/.bashrc \
    && echo "alias test-data='rostopic echo /kinefly/flystate --once'" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Show helpful commands on login" >> ~/.bashrc \
    && echo "echo '🚀 Kinefly Container Ready!'" >> ~/.bashrc \
    && echo "echo 'Command: kinefly [PORT] - Start everything (default port 9871)'" >> ~/.bashrc \
    && echo "echo 'Helpers: status | test-data'" >> ~/.bashrc

# Clean overwrite of launch folder - remove existing and copy new
RUN rm -rf /root/catkin/src/Kinefly/launch/
COPY launch/ /root/catkin/src/Kinefly/launch/
COPY kinefly.yaml /root/
# Copy ros_zmq_bridge.py to launch folder as well
COPY ros_zmq_bridge.py /root/catkin/src/Kinefly/launch/

ENTRYPOINT ["/entrypoint.sh"]
