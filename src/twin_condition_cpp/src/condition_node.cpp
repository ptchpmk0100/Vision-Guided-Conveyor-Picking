#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "twin_condition_cpp/classifier.hpp"
#include "twin_condition_cpp/crop.hpp"
#include "twin_condition_cpp/extrinsic.hpp"

namespace twin_condition
{
namespace
{

double percentile95(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const double index = 0.95 * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<size_t>(std::floor(index));
  const auto upper = static_cast<size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

}  // namespace

class ConditionNode : public rclcpp::Node
{
public:
  ConditionNode()
  : Node("parcel_condition")
  {
    const std::string model = declare_parameter<std::string>("model", "");
    const std::string extrinsic_file = declare_parameter<std::string>("extrinsic_file", "");
    const std::string detection_topic = declare_parameter<std::string>(
      "detection_topic", "/parcel_cell/moving_detections");
    min_confidence_ = declare_parameter<double>("min_confidence", 0.75);
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "/parcel_cell/condition");
    if (model.empty() || extrinsic_file.empty()) {
      throw std::runtime_error("parameters model and extrinsic_file are required");
    }

    classifier_ = std::make_unique<Classifier>(model);
    extrinsic_ = load_extrinsic(extrinsic_file);

    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/overhead_camera/camera_info", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo & message) {on_info(message);});
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/overhead_camera/image", rclcpp::QoS(2).best_effort(),
      [this](const sensor_msgs::msg::Image & message) {on_image(message);});
    detection_subscription_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      detection_topic, 10,
      [this](const vision_msgs::msg::Detection3DArray & message) {on_detections(message);});
    publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(output_topic, 10);
    timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {report();});

    std::ostringstream confidence;
    confidence << min_confidence_;
    RCLCPP_INFO(
      get_logger(), "condition classifier ready: model=%s detections=%s min_confidence=%s",
      model.c_str(), detection_topic.c_str(), confidence.str().c_str());
  }

private:
  struct Frame
  {
    double stamp;
    cv::Mat rgb;
  };

  void on_info(const sensor_msgs::msg::CameraInfo & message)
  {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        intrinsics_[row][column] = message.k[static_cast<size_t>(row * 3 + column)];
      }
    }
    have_intrinsics_ = true;
  }

  void on_image(const sensor_msgs::msg::Image & message)
  {
    if (message.encoding != "rgb8") {
      return;
    }
    cv::Mat wrapped(
      static_cast<int>(message.height), static_cast<int>(message.width), CV_8UC3,
      const_cast<unsigned char *>(message.data.data()), static_cast<size_t>(message.step));
    frames_.push_back(Frame{rclcpp::Time(message.header.stamp).seconds(), wrapped.clone()});
    while (frames_.size() > 6U) {
      frames_.pop_front();
    }
  }

  void on_detections(const vision_msgs::msg::Detection3DArray & message)
  {
    if (!have_intrinsics_ || message.detections.empty()) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    auto nearest = frames_.end();
    double nearest_delta = 0.0;
    for (auto it = frames_.begin(); it != frames_.end(); ++it) {
      const double delta = std::abs(it->stamp - stamp);
      if (nearest == frames_.end() || delta < nearest_delta) {
        nearest = it;
        nearest_delta = delta;
      }
    }
    if (nearest == frames_.end() || nearest_delta > 0.06) {
      ++unmatched_;
      return;
    }

    std::vector<cv::Mat> crops;
    std::vector<size_t> kept;
    crops.reserve(message.detections.size());
    kept.reserve(message.detections.size());
    for (size_t i = 0; i < message.detections.size(); ++i) {
      const auto & position = message.detections[i].bbox.center.position;
      const auto window = crop_window(
        {position.x, position.y, position.z}, intrinsics_, extrinsic_.rotation,
        extrinsic_.translation, nearest->rgb.rows, nearest->rgb.cols);
      if (!window) {
        ++edge_;
        continue;
      }
      crops.push_back(cut_crop(nearest->rgb, *window));
      kept.push_back(i);
    }

    vision_msgs::msg::Detection3DArray output;
    output.header = message.header;
    if (!crops.empty()) {
      const auto raw_logits = classifier_->logits(preprocess(crops), static_cast<int>(crops.size()));
      for (size_t row = 0; row < raw_logits.size(); ++row) {
        const auto probabilities = softmax({raw_logits[row][0], raw_logits[row][1]});
        const std::array<double, 2> probability_pair = {probabilities[0], probabilities[1]};
        const Decision decision = decide(probability_pair, min_confidence_);
        const auto & input = message.detections[kept[row]];
        vision_msgs::msg::Detection3D detection;
        detection.header = input.header;
        detection.id = input.id;
        detection.bbox = input.bbox;

        vision_msgs::msg::ObjectHypothesisWithPose choice;
        choice.hypothesis.class_id = decision.label;
        choice.hypothesis.score = decision.confidence;
        choice.pose.pose = detection.bbox.center;
        detection.results.push_back(choice);
        for (size_t class_index = 0; class_index < kClasses.size(); ++class_index) {
          vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
          hypothesis.hypothesis.class_id = kClasses[class_index];
          hypothesis.hypothesis.score = probabilities[class_index];
          hypothesis.pose.pose = detection.bbox.center;
          detection.results.push_back(hypothesis);
        }
        output.detections.push_back(std::move(detection));
      }
    }
    publisher_->publish(output);
    ++messages_;
    parcels_ += static_cast<int>(output.detections.size());
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    latencies_ms_.push_back(elapsed);
  }

  void report()
  {
    if (latencies_ms_.empty()) {
      return;
    }
    const double mean = std::accumulate(latencies_ms_.begin(), latencies_ms_.end(), 0.0) /
      static_cast<double>(latencies_ms_.size());
    const double maximum = *std::max_element(latencies_ms_.begin(), latencies_ms_.end());
    RCLCPP_INFO(
      get_logger(),
      "msgs=%d parcels=%d unmatched=%d edge=%d latency_ms mean=%.1f p95=%.1f max=%.1f",
      messages_, parcels_, unmatched_, edge_, mean, percentile95(latencies_ms_), maximum);
    if (latencies_ms_.size() > 200U) {
      latencies_ms_.erase(latencies_ms_.begin(), latencies_ms_.end() - 200);
    }
  }

  Mat3 intrinsics_{};
  bool have_intrinsics_{false};
  Extrinsic extrinsic_{};
  double min_confidence_{0.75};
  std::unique_ptr<Classifier> classifier_;
  std::deque<Frame> frames_;
  int messages_{0};
  int parcels_{0};
  int unmatched_{0};
  int edge_{0};
  std::vector<double> latencies_ms_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace twin_condition

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<twin_condition::ConditionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
