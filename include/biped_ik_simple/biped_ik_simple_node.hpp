// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BIPED_IK_SIMPLE__BIPED_IK_SIMPLE_NODE_HPP_
#define BIPED_IK_SIMPLE__BIPED_IK_SIMPLE_NODE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "biped_interfaces/msg/sole_poses.hpp"
#include "mx_joint_controller_msgs/msg/joint_command.hpp"
#include "IKWalk/HumanoidModel.hpp"

class BipedIKSimpleNode : public rclcpp::Node
{
public:
  BipedIKSimpleNode();

private:
  std::unique_ptr<Leph::HumanoidModel> model;

  rclcpp::Subscription<biped_interfaces::msg::SolePoses>::SharedPtr sub_sole_poses;

  // TODO(ijnek): Generalise this for other JointCommands
  rclcpp::Publisher<mx_joint_controller_msgs::msg::JointCommand>::SharedPtr pub_joints;

  std::tuple<Eigen::Vector3d, Eigen::Vector3d> convertPoseToVectors(
    const geometry_msgs::msg::Pose & pose, bool is_left_foot);
  mx_joint_controller_msgs::msg::JointCommand convertToJointCommand(
    const Rhoban::IKWalkOutputs & positions);

  float z_neutral;
  float y_neutral_abs;
};


#endif  // BIPED_IK_SIMPLE__BIPED_IK_SIMPLE_NODE_HPP_
