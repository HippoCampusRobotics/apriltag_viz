// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA
#include "apriltag_viz_node.hpp"

namespace apriltag_viz {
ApriltagVizNode::ApriltagVizNode(const rclcpp::NodeOptions &_options)
    : Node("apriltag_viz", _options) {
  InitParameters();
  InitPublishers();
  InitSubscriptions();
}

void ApriltagVizNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "tag_detections_image";
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(topic, qos);
}

std::array<double, 2> ApriltagVizNode::ProjectOntoImage(
    const std::array<double, 9> &_homography,
    const std::array<double, 2> &_point) {
  std::array<double, 2> projected_point;
  const double z = _homography[3 * 2 + 0] * _point[0] +
                   _homography[3 * 2 + 1] * _point[1] + _homography[3 * 2 + 2];
  for (int i = 0; i < 2; ++i) {
    projected_point[i] =
        (_homography[3 * i + 0] * _point[0] +
         _homography[3 * i + 1] * _point[1] + _homography[3 * i + 2]) /
        z;
  }
  return projected_point;
}

void ApriltagVizNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "image_rect";
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { OnImage(msg); });

  topic = "detections";
  detections_sub_ =
      create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
          topic, qos,
          [this](
              const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
            OnDetections(msg);
          });
}

void ApriltagVizNode::OnImage(const sensor_msgs::msg::Image::SharedPtr _msg) {
  input_image_ = cv_bridge::toCvCopy(_msg)->image;

  if (overlay_.empty()) {
    output_image_ = input_image_;
  } else {
    cv::addWeighted(input_image_, 1, overlay_, params_.alpha, 0, output_image_);
  }
  image_pub_->publish(
      *cv_bridge::CvImage(_msg->header, _msg->encoding, output_image_)
           .toImageMsg());
}
void ApriltagVizNode::OnDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr _msg) {
  if (input_image_.empty()) {
    return;
  }

  overlay_ = cv::Mat(input_image_.size(), CV_8UC3, cv::Scalar(0, 0, 0, 0));
  for (const auto &detection : _msg->detections) {
    const std::array<double, 2> center =
        ProjectOntoImage(detection.homography, {0.0, 0.0});
    const std::array<double, 2> p0 =
        ProjectOntoImage(detection.homography, {1.0, 0.0});
    const std::array<double, 2> p1 =
        ProjectOntoImage(detection.homography, {0.0, 1.0});
    cv::line(overlay_, cv::Point2d(center.at(0), center.at(1)),
             cv::Point2d(p0.at(0), p0.at(1)), cv::Scalar(255, 0, 0, 255), 3);
    cv::line(overlay_, cv::Point2d(center.at(0), center.at(1)),
             cv::Point2d(p1.at(0), p1.at(1)), cv::Scalar(0, 255, 0, 255), 3);
    for (int i = 0; i < 4; ++i) {
      cv::circle(
          overlay_,
          cv::Point(detection.corners.at(i).x, detection.corners.at(i).y), 5,
          corner_colors_.at(i), 2);
    }
  }
}

}  // namespace apriltag_viz
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_viz::ApriltagVizNode)
