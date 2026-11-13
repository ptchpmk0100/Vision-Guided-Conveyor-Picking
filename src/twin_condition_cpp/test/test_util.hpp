// Helpers for the golden-data tests (rapidjson + OpenCV only).
#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rapidjson/document.h>

#include "twin_condition_cpp/crop.hpp"

namespace test_util
{

inline rapidjson::Document load_golden()
{
  std::ifstream in(std::string(FIXTURE_DIR) + "/golden.json");
  if (!in) {throw std::runtime_error("cannot open golden.json");}
  std::stringstream ss;
  ss << in.rdbuf();
  rapidjson::Document doc;
  doc.Parse(ss.str().c_str());
  if (doc.HasParseError()) {throw std::runtime_error("golden.json parse error");}
  return doc;
}

inline std::vector<unsigned char> base64_decode(const std::string & text)
{
  static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<unsigned char> out;
  int val = 0, bits = -8;
  for (unsigned char c : text) {
    if (c == '=') {break;}
    const auto pos = chars.find(static_cast<char>(c));
    if (pos == std::string::npos) {continue;}
    val = (val << 6) + static_cast<int>(pos);
    bits += 6;
    if (bits >= 0) {
      out.push_back(static_cast<unsigned char>((val >> bits) & 0xFF));
      bits -= 8;
    }
  }
  return out;
}

inline cv::Mat crop_from_base64(const std::string & text)
{
  const auto bytes = base64_decode(text);
  cv::Mat crop(twin_condition::kCropSize, twin_condition::kCropSize, CV_8UC3);
  if (bytes.size() != crop.total() * crop.elemSize()) {throw std::runtime_error("bad crop size");}
  std::copy(bytes.begin(), bytes.end(), crop.data);
  return crop;
}

inline cv::Mat read_rgb(const std::string & file)
{
  cv::Mat bgr = cv::imread(std::string(FIXTURE_DIR) + "/" + file, cv::IMREAD_COLOR);
  if (bgr.empty()) {throw std::runtime_error("cannot read " + file);}
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

inline twin_condition::Mat3 mat3(const rapidjson::Value & v)
{
  twin_condition::Mat3 m{};
  for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) {m[i][j] = v[i][j].GetDouble();}}
  return m;
}

inline twin_condition::Vec3 vec3(const rapidjson::Value & v)
{
  return {v[0].GetDouble(), v[1].GetDouble(), v[2].GetDouble()};
}

}  // namespace test_util
