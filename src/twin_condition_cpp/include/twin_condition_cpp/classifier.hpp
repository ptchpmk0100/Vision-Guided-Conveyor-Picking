// ONNX Runtime wrapper and the decision rule of the condition node.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace twin_condition
{

inline constexpr std::array<const char *, 2> kClasses = {"intact", "damaged"};

struct Decision
{
  std::string label;   // "intact", "damaged" or "unknown"
  double confidence;   // probability of the best class
};

// argmax class, "unknown" when its probability is below min_confidence (same as the Python node).
Decision decide(const std::array<double, 2> & probabilities, double min_confidence);

class Classifier
{
public:
  // Loads model.onnx (input "input" N x 3 x 128 x 128 float, output "logits" N x 2). CPU only.
  // Throws std::runtime_error when the file cannot be loaded.
  explicit Classifier(const std::string & model_path);
  ~Classifier();
  Classifier(const Classifier &) = delete;
  Classifier & operator=(const Classifier &) = delete;

  // `nchw` holds n crops (n*3*128*128 floats). Returns n rows of {logit_intact, logit_damaged}.
  std::vector<std::array<float, 2>> logits(const std::vector<float> & nchw, int n) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace twin_condition
