// Crop geometry shared by the node and the tests. Pure functions: no ROS. Mirrors
// src/twin_moveit_scripts/twin_moveit_scripts/parcel_condition.py exactly (see plans/phase_B.md, B1.3).
#pragma once

#include <array>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace twin_condition
{

constexpr double kCropWindowM = 0.12;  // world window around the parcel centre (m)
constexpr int kCropSize = 64;          // stored crop edge (px)
constexpr int kInputSize = 128;        // model input edge (px)

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;  // row-major

struct Pixel
{
  double u;
  double v;
  double z;  // depth along the optical axis (m)
};

struct Window
{
  int u0;
  int v0;
  int u1;
  int v1;
  bool operator==(const Window & o) const {return u0 == o.u0 && v0 == o.v0 && u1 == o.u1 && v1 == o.v1;}
};

// World point -> pixel. rotation/translation are optical->world (p_w = R p_o + t). Empty when z <= 1e-6.
std::optional<Pixel> world_to_pixel(const Vec3 & point, const Mat3 & intrinsics, const Mat3 & rotation,
  const Vec3 & translation);

// Integer pixel window (half-to-even rounding, like Python round()); empty when it leaves the image
// (u0 < 0, v0 < 0, u1 > cols or v1 > rows) or the point is behind the camera.
std::optional<Window> crop_window(const Vec3 & centre, const Mat3 & intrinsics, const Mat3 & rotation,
  const Vec3 & translation, int rows, int cols);

// rgb: CV_8UC3 in RGB order. Returns a size x size CV_8UC3 crop. INTER_AREA when the window is wider than
// `size`, INTER_LINEAR otherwise (same as Python).
cv::Mat cut_crop(const cv::Mat & rgb, const Window & window, int size = kCropSize);

// Batch of CV_8UC3 crops -> float32 NCHW tensor (N*3*size*size values): bilinear resize to size x size,
// /255, minus ImageNet mean, divided by ImageNet std.
std::vector<float> preprocess(const std::vector<cv::Mat> & crops, int size = kInputSize);

// Numerically stable softmax of one row of logits.
std::vector<double> softmax(const std::vector<double> & logits);

}  // namespace twin_condition
