// ONNX Runtime wrapper and decision rule against golden logits. Skipped when CONDITION_MODEL is unset
// (CI downloads the released model and sets it).
#include <gtest/gtest.h>

#include <cstdlib>

#include "test_util.hpp"
#include "twin_condition_cpp/classifier.hpp"
#include "twin_condition_cpp/crop.hpp"

using namespace twin_condition;

TEST(Decision, ThresholdAndUnknown)
{
  EXPECT_EQ(decide({0.5, 0.5}, 0.75).label, "unknown");
  EXPECT_EQ(decide({0.9, 0.1}, 0.75).label, "intact");
  EXPECT_EQ(decide({0.1, 0.9}, 0.75).label, "damaged");
  EXPECT_EQ(decide({0.25, 0.75}, 0.75).label, "damaged");  // p >= threshold is accepted
  EXPECT_NEAR(decide({0.1, 0.9}, 0.75).confidence, 0.9, 1e-12);
}

TEST(Classifier, MissingModelThrows)
{
  EXPECT_THROW(Classifier("/nonexistent/model.onnx"), std::runtime_error);
}

TEST(Classifier, LogitsMatchPython)
{
  const char * model = std::getenv("CONDITION_MODEL");
  if (model == nullptr || std::string(model).empty()) {GTEST_SKIP() << "CONDITION_MODEL not set";}
  const auto g = test_util::load_golden();
  Classifier classifier(model);
  int n = 0;
  for (const auto & f : g["frames"].GetArray()) {
    for (const auto & d : f["detections"].GetArray()) {
      const auto tensor = preprocess({test_util::crop_from_base64(d["crop_b64"].GetString())});
      const auto logits = classifier.logits(tensor, 1);
      ASSERT_EQ(logits.size(), 1u);
      for (int c = 0; c < 2; ++c) {EXPECT_NEAR(logits[0][c], d["logits"][c].GetDouble(), 1e-3);}
      const auto p = softmax({logits[0][0], logits[0][1]});
      const auto decision = decide({p[0], p[1]}, g["min_confidence"].GetDouble());
      EXPECT_EQ(decision.label, d["label"].GetString());
      EXPECT_NEAR(decision.confidence, d["confidence"].GetDouble(), 1e-3);
      ++n;
    }
  }
  EXPECT_GE(n, 5);
}

TEST(Classifier, BatchEqualsSingles)
{
  const char * model = std::getenv("CONDITION_MODEL");
  if (model == nullptr || std::string(model).empty()) {GTEST_SKIP() << "CONDITION_MODEL not set";}
  const auto g = test_util::load_golden();
  Classifier classifier(model);
  std::vector<cv::Mat> crops;
  for (const auto & f : g["frames"].GetArray()) {
    for (const auto & d : f["detections"].GetArray()) {
      crops.push_back(test_util::crop_from_base64(d["crop_b64"].GetString()));
    }
  }
  const auto batch = classifier.logits(preprocess(crops), static_cast<int>(crops.size()));
  ASSERT_EQ(batch.size(), crops.size());
  for (size_t i = 0; i < crops.size(); ++i) {
    const auto one = classifier.logits(preprocess({crops[i]}), 1);
    EXPECT_NEAR(batch[i][0], one[0][0], 1e-4);
    EXPECT_NEAR(batch[i][1], one[0][1], 1e-4);
  }
}
