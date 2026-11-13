// Strict loader for config/overhead_camera_extrinsic.yaml (p_world = R p_optical + t).
#pragma once

#include <string>

#include "twin_condition_cpp/crop.hpp"

namespace twin_condition
{

struct Extrinsic
{
  Mat3 rotation;
  Vec3 translation;
};

// Throws std::runtime_error ("invalid extrinsic file ...") when the file is unreadable, lacks a field,
// has transform.parent_frame != "world" or transform.child_frame != "camera_optical_link", or the rotation
// is not 3x3 or the translation is not 3 numbers (same checks as _load_extrinsic in Python).
Extrinsic load_extrinsic(const std::string & path);

}  // namespace twin_condition
