#include "twin_condition_cpp/extrinsic.hpp"

#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace twin_condition
{
namespace
{

[[noreturn]] void invalid(const std::string & path, const std::string & detail)
{
  throw std::runtime_error("invalid extrinsic file " + path + ": " + detail);
}

}  // namespace

Extrinsic load_extrinsic(const std::string & path)
{
  try {
    const YAML::Node document = YAML::LoadFile(path);
    const YAML::Node transform = document["transform"];
    if (!transform || transform["parent_frame"].as<std::string>() != "world" ||
      transform["child_frame"].as<std::string>() != "camera_optical_link")
    {
      invalid(path, "expected world <- camera_optical_link");
    }
    const YAML::Node rotation_node = transform["rotation_matrix"];
    const YAML::Node translation_node = transform["translation"];
    if (!rotation_node.IsSequence() || rotation_node.size() != 3) {
      invalid(path, "rotation_matrix must be 3x3");
    }
    Extrinsic result{};
    for (size_t i = 0; i < 3; ++i) {
      if (!rotation_node[i].IsSequence() || rotation_node[i].size() != 3) {
        invalid(path, "rotation_matrix must be 3x3");
      }
      for (size_t j = 0; j < 3; ++j) {
        result.rotation[i][j] = rotation_node[i][j].as<double>();
        if (!std::isfinite(result.rotation[i][j])) {
          invalid(path, "non-finite rotation value");
        }
      }
    }
    if (!translation_node.IsSequence() || translation_node.size() != 3) {
      invalid(path, "translation must have 3 numbers");
    }
    for (size_t i = 0; i < 3; ++i) {
      result.translation[i] = translation_node[i].as<double>();
      if (!std::isfinite(result.translation[i])) {
        invalid(path, "non-finite translation value");
      }
    }

    double orthonormality_squared = 0.0;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        double value = 0.0;
        for (int k = 0; k < 3; ++k) {
          value += result.rotation[k][i] * result.rotation[k][j];
        }
        value -= i == j ? 1.0 : 0.0;
        orthonormality_squared += value * value;
      }
    }
    const auto & r = result.rotation;
    const double determinant =
      r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
      r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
      r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if (std::sqrt(orthonormality_squared) > 1e-6) {
      invalid(path, "non-orthonormal rotation");
    }
    if (std::abs(determinant - 1.0) > 1e-6) {
      invalid(path, "improper rotation");
    }
    return result;
  } catch (const std::runtime_error & error) {
    if (std::string(error.what()).rfind("invalid extrinsic file ", 0) == 0) {
      throw;
    }
    invalid(path, error.what());
  } catch (const std::exception & error) {
    invalid(path, error.what());
  }
}

}  // namespace twin_condition
