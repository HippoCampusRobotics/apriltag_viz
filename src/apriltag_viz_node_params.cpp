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
#include <hippo_common/param_utils.hpp>

#include "apriltag_viz_node.hpp"
namespace apriltag_viz {
void ApriltagVizNode::InitParameters() {
  HIPPO_COMMON_DECLARE_PARAM_DOUBLE_RANGE(alpha, 0.0, 1.0);
  HIPPO_COMMON_DECLARE_PARAM_INTEGER_RANGE(line_thickness, 1, 20, 1);
  on_set_parameters_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnSetParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult ApriltagVizNode::OnSetParameters(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "unhandled";
  std::string text;
  bool updated{false};

  for (const auto &parameter : params) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(alpha, updated, text);
  }
  if (updated) {
    RCLCPP_INFO(get_logger(), "Parameters updated.");
  }
  result.reason = text;
  return result;
}
}  // namespace apriltag_viz
