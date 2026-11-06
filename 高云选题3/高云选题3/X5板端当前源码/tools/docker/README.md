# Isolated ROS 2 build environment

This image uses the official ROS Humble Ubuntu 22.04 image mirrored by Docker's public ECR repository. The base digest is pinned. It adds cv_bridge, OpenCV, NumPy, serial, colcon and pytest inside the container only; it does not alter the Windows training Python environment or the WSL host package set.

From the workspace root in a Linux shell with Docker:

```bash
docker build -f tools/docker/Dockerfile.humble-check \
  -t rescue-ros2-humble-check:20261009 tools/docker
docker run --rm --mount type=bind,source="$(pwd)",target=/workspace \
  rescue-ros2-humble-check:20261009 bash tools/docker/build_workspace.sh
```

The build script places CMake, colcon and installation artifacts under `/tmp` in the container, then checks Python imports of generated ROS interfaces and the vision package. Source files stay on the mounted workspace. To run a launch/integration check after building, keep the same container, source `/tmp/rescue_ros_install/setup.bash` and start the required nodes.

This is an **x86_64 software integration environment**. It does not include the X5 BPU libraries, access a physical camera, verify ARM runtime compatibility, or measure X5 inference throughput. Those checks must run on the real board. No ROS daemon is installed on the Windows host.
