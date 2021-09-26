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

#include <memory>
#include <tuple>
#include "biped_ik_simple/biped_ik_simple_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

BipedIKSimpleNode::BipedIKSimpleNode()
: Node("BipedIKSimpleNode")
{
  // Parameters
  float distHipToKnee = this->declare_parameter("distHipToKnee", 0.1415);
  float distKneeToAnkle = this->declare_parameter("distKneeToAnkle", 0.1415);
  float distAnkleToGround = this->declare_parameter("distAnkleToGround", 0.059);
  float distFeetLateral = this->declare_parameter("distFeetLateral", 0.075);

  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  distHipToKnee : %f", distHipToKnee);
  RCLCPP_DEBUG(get_logger(), "  distKneeToAnkle : %f", distKneeToAnkle);
  RCLCPP_DEBUG(get_logger(), "  distAnkleToGround : %f", distAnkleToGround);
  RCLCPP_DEBUG(get_logger(), "  distFeetLateral : %f", distFeetLateral);

  // Subscriptions
  sub_ankle_poses =
    create_subscription<biped_interfaces::msg::AnklePoses>(
    "motion/ankle_poses", 1,
    [this](biped_interfaces::msg::AnklePoses::SharedPtr ankle_poses) {
      RCLCPP_DEBUG(get_logger(), "Received AnklePoses");

      Rhoban::IKWalkOutputs outputs;

      // Run inverse invert kinematics on left leg
      RCLCPP_DEBUG(get_logger(), "Calculating Left Leg IK");
      auto [posLeft, angleLeft] = convertPoseToVectors(ankle_poses->l_ankle, true);
      bool successLeft = model->legIkLeft(
        posLeft, angleLeft, Leph::EulerRollPitchYaw, outputs);
      if (!successLeft) {
        RCLCPP_ERROR(get_logger(), "Left Invesrse Kinematics Failed, can't reach position.");
      }

      // Run inverse invert kinematics on right leg
      RCLCPP_DEBUG(get_logger(), "Calculating Right Leg IK");
      auto [posRight, angleRight] = convertPoseToVectors(ankle_poses->r_ankle, false);
      bool successRight = model->legIkRight(
        posRight, angleRight, Leph::EulerRollPitchYaw, outputs);
      if (!successRight) {
        RCLCPP_ERROR(get_logger(), "Right Invesrse Kinematics Failed, can't reach position.");
      }

      RCLCPP_DEBUG(get_logger(), "Finished IK, publishing joint commands");
      pub_joints->publish(convertToJointCommand(outputs));
    });

  // Publishers
  // TODO(ijnek): Generalise this to different JointCommands, not specific to bold bots.
  pub_joints =
    this->create_publisher<mx_joint_controller_msgs::msg::JointCommand>(
    "/cm730/joint_commands",
    10);

  // Init Humanoid Model
  model = std::make_unique<Leph::HumanoidModel>(
    distHipToKnee, distKneeToAnkle, distAnkleToGround,
    distFeetLateral);

  // Calculate some neutral values.
  z_neutral = -distHipToKnee - distKneeToAnkle;
  y_neutral_abs = distFeetLateral / 2;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> BipedIKSimpleNode::convertPoseToVectors(
  const geometry_msgs::msg::Pose & pose, bool is_left_foot)
{
  // Convert some position
  double x = pose.position.x;
  double y = pose.position.y - y_neutral_abs * (is_left_foot ? 1 : -1);
  double z = pose.position.z - z_neutral;

  // Convert Quaternion to Euler
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_DEBUG(
    get_logger(), "converted to RPY: (%f, %f, %f)",
    roll, pitch, yaw);

  return {Eigen::Vector3d{x, y, z}, Eigen::Vector3d{roll, pitch, yaw}};
}

mx_joint_controller_msgs::msg::JointCommand BipedIKSimpleNode::convertToJointCommand(
  const Rhoban::IKWalkOutputs & positions)
{
  mx_joint_controller_msgs::msg::JointCommand jointMsg;
  jointMsg.name.push_back("hip-yaw-l");
  jointMsg.position.push_back(positions.left_hip_yaw);
  jointMsg.name.push_back("hip-roll-l");
  jointMsg.position.push_back(positions.left_hip_roll);
  jointMsg.name.push_back("hip-pitch-l");
  jointMsg.position.push_back(-1 * positions.left_hip_pitch);
  jointMsg.name.push_back("knee-l");
  jointMsg.position.push_back(-1 * positions.left_knee);
  jointMsg.name.push_back("ankle-pitch-l");
  jointMsg.position.push_back(positions.left_ankle_pitch);
  jointMsg.name.push_back("ankle-roll-l");
  jointMsg.position.push_back(positions.left_ankle_roll);

  jointMsg.name.push_back("hip-yaw-r");
  jointMsg.position.push_back(positions.right_hip_yaw);
  jointMsg.name.push_back("hip-roll-r");
  jointMsg.position.push_back(positions.right_hip_roll);
  jointMsg.name.push_back("hip-pitch-r");
  jointMsg.position.push_back(positions.right_hip_pitch);
  jointMsg.name.push_back("knee-r");
  jointMsg.position.push_back(positions.right_knee);
  jointMsg.name.push_back("ankle-pitch-r");
  jointMsg.position.push_back(-1 * positions.right_ankle_pitch);
  jointMsg.name.push_back("ankle-roll-r");
  jointMsg.position.push_back(positions.right_ankle_roll);

  return jointMsg;
}
