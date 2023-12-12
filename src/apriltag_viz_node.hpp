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

#pragma once
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace apriltag_viz {

class ApriltagVizNode : public rclcpp::Node {
 public:
  explicit ApriltagVizNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  struct Params {
    double alpha;
    int line_thickness; 
  };
  void InitPublishers();
  void InitSubscriptions();
  void InitParameters();
  std::array<double, 2> ProjectOntoImage(
      const std::array<double, 9> &homography,
      const std::array<double, 2> &point);

  void OnImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnDetections(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      detections_sub_;

  rcl_interfaces::msg::SetParametersResult OnSetParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;

  Params params_;
  cv::Mat input_image_;
  cv::Mat overlay_;
  cv::Mat output_image_;

  const std::array<cv::Scalar, 4> corner_colors_{
      {cv::Scalar(0, 0, 255, 255), cv::Scalar(0, 255, 0, 255),
       cv::Scalar(255, 0, 0, 255), cv::Scalar(0, 255, 255, 255)}};
};

}  // namespace apriltag_viz
