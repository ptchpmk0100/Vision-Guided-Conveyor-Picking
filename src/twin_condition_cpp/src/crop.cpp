#include "twin_condition_cpp/crop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace twin_condition
{

std::optional<Pixel> world_to_pixel(
  const Vec3 & point, const Mat3 & intrinsics, const Mat3 & rotation,
  const Vec3 & translation)
{
  Vec3 optical{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      optical[i] += rotation[j][i] * (point[j] - translation[j]);
    }
  }
  const double z = optical[2];
  if (z <= 1e-6) {
    return std::nullopt;
  }
  return Pixel{
    intrinsics[0][0] * optical[0] / z + intrinsics[0][2],
    intrinsics[1][1] * optical[1] / z + intrinsics[1][2], z};
}

std::optional<Window> crop_window(
  const Vec3 & centre, const Mat3 & intrinsics, const Mat3 & rotation,
  const Vec3 & translation, int rows, int cols)
{
  const auto pixel = world_to_pixel(centre, intrinsics, rotation, translation);
  if (!pixel) {
    return std::nullopt;
  }
  const double half = kCropWindowM / 2.0 * intrinsics[0][0] / pixel->z;
  const Window window{
    static_cast<int>(std::nearbyint(pixel->u - half)),
    static_cast<int>(std::nearbyint(pixel->v - half)),
    static_cast<int>(std::nearbyint(pixel->u + half)),
    static_cast<int>(std::nearbyint(pixel->v + half))};
  if (window.u0 < 0 || window.v0 < 0 || window.u1 > cols || window.v1 > rows) {
    return std::nullopt;
  }
  return window;
}

cv::Mat cut_crop(const cv::Mat & rgb, const Window & window, int size)
{
  const cv::Rect roi(window.u0, window.v0, window.u1 - window.u0, window.v1 - window.v0);
  cv::Mat resized;
  cv::resize(
    rgb(roi), resized, cv::Size(size, size), 0.0, 0.0,
    (window.u1 - window.u0) > size ? cv::INTER_AREA : cv::INTER_LINEAR);
  return resized;
}

std::vector<float> preprocess(const std::vector<cv::Mat> & crops, int size)
{
  constexpr float mean[3] = {0.485F, 0.456F, 0.406F};
  constexpr float stddev[3] = {0.229F, 0.224F, 0.225F};
  const size_t plane = static_cast<size_t>(size) * static_cast<size_t>(size);
  std::vector<float> output(crops.size() * 3U * plane);
  for (size_t n = 0; n < crops.size(); ++n) {
    cv::Mat resized;
    cv::resize(crops[n], resized, cv::Size(size, size), 0.0, 0.0, cv::INTER_LINEAR);
    for (int y = 0; y < size; ++y) {
      const auto * row = resized.ptr<cv::Vec3b>(y);
      for (int x = 0; x < size; ++x) {
        for (int c = 0; c < 3; ++c) {
          const size_t index = n * 3U * plane + static_cast<size_t>(c) * plane +
            static_cast<size_t>(y) * static_cast<size_t>(size) + static_cast<size_t>(x);
          output[index] = (static_cast<float>(row[x][c]) / 255.0F - mean[c]) / stddev[c];
        }
      }
    }
  }
  return output;
}

std::vector<double> softmax(const std::vector<double> & logits)
{
  if (logits.empty()) {
    return {};
  }
  const double maximum = *std::max_element(logits.begin(), logits.end());
  std::vector<double> probabilities(logits.size());
  double total = 0.0;
  for (size_t i = 0; i < logits.size(); ++i) {
    probabilities[i] = std::exp(logits[i] - maximum);
    total += probabilities[i];
  }
  for (double & probability : probabilities) {
    probability /= total;
  }
  return probabilities;
}

}  // namespace twin_condition
