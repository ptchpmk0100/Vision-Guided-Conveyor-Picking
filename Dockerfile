# Headless image for the offline test suite and the C++ condition node's gtests (no Gazebo, no GUI).
#   docker build -t condition-cell .
#   docker run --rm condition-cell          -> ends with "C++ TESTS: ALL PASS" and "OFFLINE SUITE: ALL PASS"
# Installs through tools/install_ci_deps.sh, the same script CI uses.
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]
COPY tools/install_ci_deps.sh /tmp/install_ci_deps.sh
RUN bash /tmp/install_ci_deps.sh --dest /opt/dl

WORKDIR /ws
COPY . /ws
RUN source /opt/ros/jazzy/setup.bash && source /opt/dl/env.sh && \
    colcon build --base-paths src --packages-select twin_description twin_moveit_config twin_condition_cpp

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /opt/dl/env.sh && source install/setup.bash && tools/check_condition_cpp.sh && tools/run_offline_tests.sh"]
