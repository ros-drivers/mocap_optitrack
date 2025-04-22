FROM ros:foxy as builder
LABEL org.opencontainers.image.authors="Jan-Hendrik Ewers <me@janhendrikewers.uk>"
LABEL org.opencontainers.image.version="1.0.0"
LABEL org.opencontainers.image.documentation="https://github.com/ros-drivers/mocap_optitrack"
LABEL org.opencontainers.image.description="Run mocap_optitrack using ros2 foxy"

# Copy the source code
WORKDIR /ws/src/mocap_optitrack
COPY . .

# Build mocap_optitrack
WORKDIR /ws/
RUN /ros_entrypoint.sh bash -c \
        "rosdep update && rosdep install --from-paths src --ignore-src -y && colcon build"

FROM ros:foxy as runner

# No need to keep the source code within the image
COPY --from=builder /ws/install /ws/install

# Update /ros_entrypoint.sh to source /ws/install/setup.bash
RUN echo '#!/bin/bash\nset -e\nsource "/opt/ros/$ROS_DISTRO/setup.bash" --\nsource "/ws/install/setup.bash" --\nexec "$@"\n' > /ros_entrypoint.sh

WORKDIR /ws

# Launch mocap_optitrack with:
#   docker build . -t mocap_optitrack:foxy && docker run --rm -it mocap_optitrack:foxy ros2 launch mocap_optitrack mocap.launch.py
