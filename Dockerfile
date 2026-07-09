# Start from official ROS 2 Jazzy
FROM ros:jazzy

# 1. Install general development tools (things not usually in package.xml)
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /root

# 2. THE SECRET STEP: Copy your local 'src' folder into the image during the build
# This gives rosdep access to your package.xml files so it knows what to install!
COPY seaplane_ws /root/seaplane_ws

# 3. Scan package.xml and install all missing ROS/system dependencies automatically
RUN apt-get update && rosdep update && \
    rosdep install --from-paths seaplane_ws/src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# 4. Automatically source base ROS 2 and create the 'sros' alias for your workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "alias sros='source install/setup.bash'" >> ~/.bashrc

CMD ["bash"]
