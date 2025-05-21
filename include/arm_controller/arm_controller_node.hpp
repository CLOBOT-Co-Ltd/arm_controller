#ifndef ARM_CONTROLLER__ARM_CONTROLLER_NODE_HPP_
#define ARM_CONTROLLER__ARM_CONTROLLER_NODE_HPP_


// Copyright 2025 clober
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "arm_controller/definitions.hpp"
#include "arm_controller/iarm_controller.hpp"
#include "arm_controller/controller_FSM.hpp"

#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#include <chrono>
#include <iostream>
#include <functional>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "clobot_platform_libs/thread/timer.hpp"

#include "std_msgs/msg/bool.hpp"
#include "arm_interfaces/msg/joint_info_list.hpp"
#include "arm_interfaces/action/gesture.hpp"


using namespace std::chrono_literals;


class ArmControllerNode : public rclcpp::Node, public IArmController
{
private:
  std::atomic<bool> arm_emergency_stop_flag_ = false;
  std::atomic<bool> arm_test_flag_ = false;
  std::atomic<bool> gesture_action_flag_ = false;
  uint8_t gesture_type_ = 0;

  JOINT_INFOS arm_joint_infos_ = {0};

  std::shared_ptr<ControllerFSM> controller_FSM_;
  clobot_platform_libs::thread::Timer controller_FSM_timer_; // 20 ms

  // control parameters
  double controller_freq_hz_ = 50.0; // hz

  double max_angular_vel_rps_ = 0.5; // rad/sec
  double max_angle_delta_rad_ = 0.01; // radian
  double min_angle_delta_rad_ = 0.001; // radian

  double angle_tolerance_rad_; // 모터 각도 허용 오차 (rad)

  double weight_rate_;
  // control parameters


  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
  arm_sdk_publisher_;

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
  low_state_subscriber_;

  rclcpp::Publisher<arm_interfaces::msg::JointInfoList>::SharedPtr pub_arm_joint_infos_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_emergency_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_test_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_interfaces::action::Gesture>>
  action_handler_gesture_;

  rclcpp_action::Server<arm_interfaces::action::Gesture>::SharedPtr action_server_gesture_;

public:
  ArmControllerNode(const char * networkInterface);
// ~ArmControllerNode();

private:
// 각 필드 값 초기화
  void initialize(const char * networkInterface);

// FSM 가동 타이머의 실행 메소드
  void on_controller_FSM_timer_elapsed(void * nothing);

// unitree dds 토픽 콜백 함수
  void on_dds_subscribed_rt_lowstate(const void * rt_lowstate_msg);

// topic 콜백 함수
  void on_subscribed_arm_emergency_stop(
    const std_msgs::msg::Bool::SharedPtr arm_emergency_stop_msg);

  void on_subscribed_arm_test(
    const std_msgs::msg::Bool::SharedPtr arm_test_msg);

// /gesture action 콜백 함수
  rclcpp_action::GoalResponse on_received_action_goal_gesture(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const arm_interfaces::action::Gesture::Goal> action_goal_msg);

  rclcpp_action::CancelResponse on_received_action_cancel_gesture(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_interfaces::action::Gesture>>
    action_handler_gesture);

  void on_received_action_accepted_gesture(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_interfaces::action::Gesture>>
    action_handler_gesture);

public:
// IArmController 인터페이스 구현
  bool get_arm_emergency_stop() override;
  std::array<double, JOINT_NUMBER> get_arm_joint_infos() override;
  bool get_gesture_action_flag() override;
  uint8_t get_gesture_action_type() override;

  void set_arm_motor_cmd(
    std::array<double, JOINT_NUMBER> q, std::array<double, JOINT_NUMBER> dq,
    std::array<double, JOINT_NUMBER> kp, std::array<double, JOINT_NUMBER> kd,
    std::array<double, JOINT_NUMBER> tau,
    double weight = 1.0) override;

  bool is_all_topics_ready() override;
  void reset_topic_flags() override;
};

#endif // ARM_CONTROLLER__ARM_CONTROLLER_NODE_HPP_
