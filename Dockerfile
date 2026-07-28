# Use Ubuntu 16.04 as the base image
FROM ubuntu:16.04

# Keep the EOL ROS Kinetic/Python 2 dependency metadata reproducible.
# This is the last rosdistro revision before the legacy Python 2 keys were removed.
ARG ROSDISTRO_SNAPSHOT=d6b7bf33d96eee741e097d63883ae91b1b38f91f

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=kinetic \
    ROS_PYTHON_VERSION=2 \
    ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/${ROSDISTRO_SNAPSHOT}/index-v4.yaml

# Install necessary packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    sudo \
    wget \
    git \
    curl \
    python \
    python-dev \
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
        python-empy \
        python-rosdep \
        python-catkin-pkg \
        python-catkin-pkg-modules \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        ros-${ROS_DISTRO}-driver-base \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep using the pinned Python 2-compatible ruleset. The local
# definitions keep Xenial resolution explicit even if a package source changes.
RUN rosdep init \
    && sed -i "s|raw.githubusercontent.com/ros/rosdistro/master/|raw.githubusercontent.com/ros/rosdistro/${ROSDISTRO_SNAPSHOT}/|g" /etc/ros/rosdep/sources.list.d/20-default.list \
    && printf "python-catkin-pkg:\n  ubuntu:\n    xenial: [python-catkin-pkg]\npython-catkin-pkg-modules:\n  ubuntu:\n    xenial: [python-catkin-pkg-modules]\npython-empy:\n  ubuntu:\n    xenial: [python-empy]\npython-rosdep:\n  ubuntu:\n    xenial: [python-rosdep]\n" > /etc/ros/rosdep/local-python2.yaml \
    && printf "yaml file:///etc/ros/rosdep/local-python2.yaml\n" > /etc/ros/rosdep/sources.list.d/00-local-python2.list \
    && rosdep update \
    && rosdep resolve python-catkin-pkg \
    && rosdep resolve python-empy \
    && rosdep resolve python-rosdep \
    && python -c "import sys; assert sys.version_info[0] == 2, sys.version"

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

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin/devel/setup.bash && rosmake Kinefly"

# Install guvcview
RUN apt-get update && apt-get install -y \
    v4l-utils \
    guvcview \
    gedit


# Create directory for Kinefly bridge scripts
RUN mkdir -p /opt/Kinefly_docker


# Install Python dependencies for ZMQ bridge
RUN apt-get update && apt-get install -y \
    python-pip \
    && python -m pip install "click==6.7" "pyzmq==17.1.2" \
    && python -c "import sys, click, zmq; assert sys.version_info[0] == 2, sys.version" \
    && rm -rf /var/lib/apt/lists/*

# Copy essential scripts and files to the container
COPY tests/test_zmq_client.py /opt/Kinefly_docker/
COPY tests/test_camera.sh /opt/Kinefly_docker/
COPY tests/test_flystate_publisher.py /opt/Kinefly_docker/
COPY kinefly /opt/Kinefly_docker/
COPY _internal/ /opt/Kinefly_docker/
RUN chmod +x /opt/Kinefly_docker/kinefly /opt/Kinefly_docker/*.sh /opt/Kinefly_docker/test_camera.sh 2>/dev/null || true


# Final setup steps
RUN echo "export RIG=VR1" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source /root/catkin/devel/setup.bash && rosmake Kinefly"

# Enhanced environment setup - Add all necessary environment variables and aliases
RUN echo "" >> ~/.bashrc \
    && echo "# === Kinefly Environment Setup ===" >> ~/.bashrc \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin/devel/setup.bash" >> ~/.bashrc \
    && echo "export RIG=VR1" >> ~/.bashrc \
    && echo "export PYTHONPATH=/root/catkin/src/Kinefly/src:\$PYTHONPATH" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# === Simple Aliases ===" >> ~/.bashrc \
    && echo "alias kinefly='/opt/Kinefly_docker/kinefly'" >> ~/.bashrc \
    && echo "alias kinefly-cam1='/opt/Kinefly_docker/start-kinefly-cam1.sh'" >> ~/.bashrc \
    && echo "alias kinefly-cam2='/opt/Kinefly_docker/start-kinefly-cam2.sh'" >> ~/.bashrc \
    && echo "alias kinefly-dual='/opt/Kinefly_docker/start-kinefly-dual.sh'" >> ~/.bashrc \
    && echo "alias status='rostopic list | grep -E \"(kinefly|camera)\"'" >> ~/.bashrc \
    && echo "alias test-data='rostopic echo /kinefly/flystate --once'" >> ~/.bashrc \
    && echo "alias test-cam1='rostopic echo /kinefly_cam1/flystate --once'" >> ~/.bashrc \
    && echo "alias test-cam2='rostopic echo /kinefly_cam2/flystate --once'" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Show helpful commands on login" >> ~/.bashrc \
    && echo "echo '🚀 Kinefly Container Ready!'" >> ~/.bashrc \
    && echo "echo 'Commands:'" >> ~/.bashrc \
    && echo "echo '  kinefly [VR_NUM] [PORT] - Start VR(s) (no args = all VRs, VR_NUM 1-4)'" >> ~/.bashrc \
    && echo "echo '  kinefly [PORT]         - Original single camera (default port 9871)'" >> ~/.bashrc \
    && echo "echo '  kinefly-cam1 [PORT]    - Camera 1 only (default port 9871)'" >> ~/.bashrc \
    && echo "echo '  kinefly-cam2 [PORT]    - Camera 2 only (default port 9872)'" >> ~/.bashrc \
    && echo "echo '  kinefly-dual [P1] [P2] - Both cameras (default ports 9871, 9872)'" >> ~/.bashrc \
    && echo "echo 'Helpers: status | test-data | test-cam1 | test-cam2'" >> ~/.bashrc

# Copy launch configurations and other files - this will automatically include all subdirectories
# Clean overwrite of launch folder - remove existing and copy new (this copies everything automatically)
RUN rm -rf /root/catkin/src/Kinefly/launch/
COPY launch/ /root/catkin/src/Kinefly/launch/
COPY config/kinefly.yaml /root/
# Copy ros_zmq_bridge.py to launch folder as well (from _internal/)
COPY _internal/ros_zmq_bridge.py /root/catkin/src/Kinefly/launch/

# Create configuration directories for each camera instance to prevent IOError
RUN mkdir -p /root/kinefly_cam1 /root/kinefly_cam2 \
    && cp /root/kinefly.yaml /root/kinefly_cam1/kinefly_cam1.yaml \
    && cp /root/kinefly.yaml /root/kinefly_cam2/kinefly_cam2.yaml \
    && cp /root/kinefly.png /root/kinefly_cam1/ 2>/dev/null || echo "kinefly.png not found, skipping" \
    && cp /root/kinefly.png /root/kinefly_cam2/ 2>/dev/null || echo "kinefly.png not found, skipping"

# Set default command to bash for interactive use
CMD ["/bin/bash"]
