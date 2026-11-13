#include "twin_condition_cpp/classifier.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include <onnxruntime_cxx_api.h>

#include "twin_condition_cpp/crop.hpp"

namespace twin_condition
{

Decision decide(const std::array<double, 2> & probabilities, double min_confidence)
{
  const size_t best = probabilities[1] > probabilities[0] ? 1U : 0U;
  return Decision{
    probabilities[best] >= min_confidence ? kClasses[best] : "unknown",
    probabilities[best]};
}

struct Classifier::Impl
{
  explicit Impl(const std::string & model_path)
  : env(ORT_LOGGING_LEVEL_WARNING, "twin_condition_cpp"), options(),
    session(nullptr)
  {
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = Ort::Session(env, model_path.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input = session.GetInputNameAllocated(0, allocator);
    const auto output = session.GetOutputNameAllocated(0, allocator);
    input_name = input.get();
    output_name = output.get();
  }

  Ort::Env env;
  Ort::SessionOptions options;
  Ort::Session session;
  std::string input_name;
  std::string output_name;
};

Classifier::Classifier(const std::string & model_path)
{
  try {
    impl_ = std::make_unique<Impl>(model_path);
  } catch (const Ort::Exception & error) {
    throw std::runtime_error("failed to load ONNX model " + model_path + ": " + error.what());
  }
}

Classifier::~Classifier() = default;

std::vector<std::array<float, 2>> Classifier::logits(
  const std::vector<float> & nchw, int n) const
{
  if (n < 0 || nchw.size() != static_cast<size_t>(n) * 3U * kInputSize * kInputSize) {
    throw std::runtime_error("invalid classifier input tensor size");
  }
  if (n == 0) {
    return {};
  }
  const std::array<int64_t, 4> shape = {n, 3, kInputSize, kInputSize};
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<float>(
    memory, const_cast<float *>(nchw.data()), nchw.size(), shape.data(), shape.size());
  const char * input_names[] = {impl_->input_name.c_str()};
  const char * output_names[] = {impl_->output_name.c_str()};
  try {
    auto outputs = impl_->session.Run(
      Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
    const auto info = outputs[0].GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      info.GetElementCount() != static_cast<size_t>(n) * 2U)
    {
      throw std::runtime_error("unexpected classifier output shape or type");
    }
    const float * data = outputs[0].GetTensorData<float>();
    std::vector<std::array<float, 2>> result(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      result[static_cast<size_t>(i)] = {data[i * 2], data[i * 2 + 1]};
    }
    return result;
  } catch (const Ort::Exception & error) {
    throw std::runtime_error(std::string("ONNX inference failed: ") + error.what());
  }
}

}  // namespace twin_condition
