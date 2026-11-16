// Crop geometry, preprocessing, softmax and extrinsic loader against golden data made by the Python code.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>

#include "test_util.hpp"
#include "twin_condition_cpp/crop.hpp"
#include "twin_condition_cpp/extrinsic.hpp"

using namespace twin_condition;
using test_util::load_golden;

TEST(Crop, WindowsMatchPython)
{
  const auto g = load_golden();
  const auto k = test_util::mat3(g["intrinsics"]);
  const auto r = test_util::mat3(g["rotation"]);
  const auto t = test_util::vec3(g["translation"]);
  int checked = 0;
  for (const auto & c : g["window_cases"].GetArray()) {
    const auto got = crop_window(test_util::vec3(c["centre"]), k, r, t, 480, 640);
    if (c["window"].IsNull()) {
      EXPECT_FALSE(got.has_value()) << "centre should be rejected";
    } else {
      ASSERT_TRUE(got.has_value());
      const Window want{c["window"][0].GetInt(), c["window"][1].GetInt(), c["window"][2].GetInt(),
        c["window"][3].GetInt()};
      EXPECT_TRUE(*got == want) << got->u0 << "," << got->v0 << "," << got->u1 << "," << got->v1;
    }
    ++checked;
  }
  EXPECT_GE(checked, 9);
}

TEST(Crop, RoundingIsHalfToEvenLikePython)
{
  const auto g = load_golden();
  const auto & c = g["rounding_case"];
  const auto got = crop_window(test_util::vec3(c["centre"]), test_util::mat3(c["intrinsics"]),
    test_util::mat3(c["rotation"]), test_util::vec3(c["translation"]), c["shape"][0].GetInt(),
    c["shape"][1].GetInt());
  ASSERT_TRUE(got.has_value());
  // corners fall on 94.5 and 106.5: half-to-even gives 94 / 106; lround would give 95 / 107.
  EXPECT_EQ(got->u0, 94);
  EXPECT_EQ(got->u1, 106);
}

TEST(Crop, CutCropMatchesPythonWithinOneGreyLevel)
{
  const auto g = load_golden();
  int n = 0;
  for (const auto & f : g["frames"].GetArray()) {
    const cv::Mat rgb = test_util::read_rgb(f["file"].GetString());
    for (const auto & d : f["detections"].GetArray()) {
      const Window w{d["window"][0].GetInt(), d["window"][1].GetInt(), d["window"][2].GetInt(),
        d["window"][3].GetInt()};
      const cv::Mat got = cut_crop(rgb, w);
      const cv::Mat want = test_util::crop_from_base64(d["crop_b64"].GetString());
      ASSERT_EQ(got.size(), want.size());
      ASSERT_EQ(got.type(), want.type());
      double max_diff = 0;
      cv::Mat a, b;
      got.convertTo(a, CV_16S);
      want.convertTo(b, CV_16S);
      const cv::Mat diff = cv::abs(a - b);
      cv::minMaxLoc(diff.reshape(1), nullptr, &max_diff);
      EXPECT_LE(max_diff, 1.0) << "frame " << f["file"].GetString();
      ++n;
    }
  }
  EXPECT_GE(n, 5);
}

TEST(Crop, PreprocessMatchesPython)
{
  const auto g = load_golden();
  for (const auto & f : g["frames"].GetArray()) {
    for (const auto & d : f["detections"].GetArray()) {
      const auto tensor = preprocess({test_util::crop_from_base64(d["crop_b64"].GetString())});
      ASSERT_EQ(tensor.size(), static_cast<size_t>(3 * kInputSize * kInputSize));
      for (int i = 0; i < 12; ++i) {
        EXPECT_NEAR(tensor[i], d["tensor_head"][i].GetDouble(), 1e-4) << "value " << i;
      }
      double sum = 0;
      for (float x : tensor) {sum += x;}
      EXPECT_NEAR(sum, d["tensor_sum"].GetDouble(), 0.5);
    }
  }
}

TEST(Crop, SoftmaxMatchesPython)
{
  const auto g = load_golden();
  const auto & s = g["softmax_case"];
  for (rapidjson::SizeType i = 0; i < s["logits"].Size(); ++i) {
    const auto p = softmax({s["logits"][i][0].GetDouble(), s["logits"][i][1].GetDouble()});
    ASSERT_EQ(p.size(), 2u);
    EXPECT_NEAR(p[0], s["probs"][i][0].GetDouble(), 1e-9);
    EXPECT_NEAR(p[1], s["probs"][i][1].GetDouble(), 1e-9);
    EXPECT_NEAR(p[0] + p[1], 1.0, 1e-12);
  }
}

TEST(Extrinsic, LoadsRealFileLikePython)
{
  const auto g = load_golden();
  const auto e = load_extrinsic(EXTRINSIC_FILE);
  const auto r = test_util::mat3(g["rotation"]);
  const auto t = test_util::vec3(g["translation"]);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(e.translation[i], t[i], 1e-12);
    for (int j = 0; j < 3; ++j) {EXPECT_NEAR(e.rotation[i][j], r[i][j], 1e-12);}
  }
}

TEST(Extrinsic, RejectsWrongFrameAndMissingFile)
{
  const std::string path = std::string(::testing::TempDir()) + "bad_extrinsic.yaml";
  {
    std::ofstream out(path);
    out << "transform:\n  parent_frame: base\n  child_frame: camera_optical_link\n"
           "  rotation_matrix: [[1,0,0],[0,1,0],[0,0,1]]\n  translation: [0,0,0]\n";
  }
  EXPECT_THROW(load_extrinsic(path), std::runtime_error);
  std::remove(path.c_str());
  EXPECT_THROW(load_extrinsic("/nonexistent/extrinsic.yaml"), std::runtime_error);
}
