# Reproducible environment for the Factory Sorting Cell.
# Build:  docker build -t sorting-cell .
# Run  :  docker run --rm -it sorting-cell            (headless sim needs a GPU/display setup)
# NOTE: this image is provided for reproducibility; it is NOT built or exercised inside the
# sim-only development workspace. The offline test suite is what CI runs (see .github/).
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-moveit \
      ros-jazzy-moveit-py \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      python3-opencv \
      python3-numpy \
      python3-yaml \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install \
      --packages-select twin_action_demo twin_moveit_scripts

# default: drop into a shell with the overlay sourced
RUN echo "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export GZ_VERSION=harmonic" >> /root/.bashrc
CMD ["bash"]
