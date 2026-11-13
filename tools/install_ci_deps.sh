#!/bin/bash
# Install everything the offline suite and the C++ condition node need on a plain ros:jazzy image.
# Shared by CI (.github/workflows/ci.yml) and the Dockerfile, so the two cannot drift. Run as root.
#
#   tools/install_ci_deps.sh [--no-downloads] [--dest DIR]
#
# Apt packages always. Unless --no-downloads: ONNX Runtime and the released classifier model into DIR
# (default /opt/dl), both checked by sha256, plus DIR/env.sh exporting ONNXRUNTIME_ROOT and CONDITION_MODEL.
# Under GitHub Actions the two variables are also appended to $GITHUB_ENV.
set -euo pipefail
DEST=/opt/dl; DOWNLOAD=1
while [ $# -gt 0 ]; do
  case "$1" in
    --dest) DEST="$2"; shift 2;;
    --no-downloads) DOWNLOAD=0; shift;;
    *) echo "unknown argument $1" >&2; exit 2;;
  esac
done
ORT_VERSION=1.31.1
ORT_SHA256=c849e36a1ef3e22d4ff217b9d1ce4a3cf5b9467b975610913ae959ad5b16bdc9
MODEL_URL=https://github.com/ptchpmk0100/Vision-Guided-Conveyor-Picking/releases/download/v1.0/condition-model-r1.onnx
MODEL_SHA256=f94d9d357945fca7be6b71e767fee6f1ad92e7e42d7a3c360d49ac4c81892405

export DEBIAN_FRONTEND=noninteractive
printf 'Acquire::Retries "5";\nAcquire::http::Timeout "20";\n' > /etc/apt/apt.conf.d/99-ci
apt-get update
# offline suite: the parcel-cell tests import MoveIt and the control/vision message packages (no simulator)
# C++ condition node: OpenCV, yaml-cpp, rapidjson (tests), gtest
apt-get install -y --no-install-recommends curl ca-certificates build-essential cmake \
  python3-numpy python3-opencv python3-yaml python3-colcon-common-extensions \
  libopencv-dev libyaml-cpp-dev rapidjson-dev \
  ros-jazzy-moveit-py ros-jazzy-moveit-configs-utils ros-jazzy-moveit-msgs \
  ros-jazzy-vision-msgs ros-jazzy-control-msgs ros-jazzy-trajectory-msgs \
  ros-jazzy-shape-msgs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-rosgraph-msgs ros-jazzy-std-msgs ros-jazzy-ament-cmake-gtest
rm -rf /var/lib/apt/lists/*

[ "$DOWNLOAD" = 1 ] || exit 0
mkdir -p "$DEST" && cd "$DEST"
curl -sSfL --retry 3 -o ort.tgz \
  "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz"
echo "${ORT_SHA256}  ort.tgz" | sha256sum -c -
tar -xzf ort.tgz && rm ort.tgz
curl -sSfL --retry 3 -o model.onnx "$MODEL_URL"
echo "${MODEL_SHA256}  model.onnx" | sha256sum -c -
cat > env.sh <<ENV
export ONNXRUNTIME_ROOT=$DEST/onnxruntime-linux-x64-${ORT_VERSION}
export CONDITION_MODEL=$DEST/model.onnx
ENV
if [ -n "${GITHUB_ENV:-}" ]; then sed 's/^export //' env.sh >> "$GITHUB_ENV"; fi
echo "installed: $(cat env.sh | tr '\n' ' ')"
