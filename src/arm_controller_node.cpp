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

#include "arm_controller/arm_controller_node.hpp"


ArmControllerNode::ArmControllerNode(const char * networkInterface)
: Node("arm_controller_node")
  , controller_FSM_timer_(
    std::bind(&ArmControllerNode::on_controller_FSM_timer_elapsed, this, std::placeholders::_1),
    (1 / controller_freq_hz_) * 1000) // 20 ms
{
  RCLCPP_INFO(get_logger(), "ArmControllerNode constructor called");
  initialize(networkInterface);
}
// ArmControllerNode::~ArmControllerNode()


void ArmControllerNode::initialize(const char * networkInterface)
{
  RCLCPP_INFO(get_logger(), "ArmControllerNode initialize called");

  unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
  arm_sdk_publisher_.reset(
    new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
      "rt/arm_sdk"));
  arm_sdk_publisher_->InitChannel();

  low_state_subscriber_.reset(
    new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
      "rt/lowstate"));

  low_state_subscriber_->InitChannel(
    std::bind(
      &ArmControllerNode::on_dds_subscribed_rt_lowstate, this,
      std::placeholders::_1), 1);

  // h1_2 각 joint 번호
  arm_joint_infos_.joint_number[LEFT_SHOULDER_PITCH] = 13;
  arm_joint_infos_.joint_number[LEFT_SHOULDER_ROLL] = 14;
  arm_joint_infos_.joint_number[LEFT_SHOULDER_YAW] = 15;
  arm_joint_infos_.joint_number[LEFT_ELBOW_PITCH] = 16;
  arm_joint_infos_.joint_number[LEFT_ELBOW_ROLL] = 17;
  arm_joint_infos_.joint_number[LEFT_WRIST_PITCH] = 18;
  arm_joint_infos_.joint_number[LEFT_WRIST_YAW] = 19;
  arm_joint_infos_.joint_number[RIGHT_SHOULDER_PITCH] = 20;
  arm_joint_infos_.joint_number[RIGHT_SHOULDER_ROLL] = 21;
  arm_joint_infos_.joint_number[RIGHT_SHOULDER_YAW] = 22;
  arm_joint_infos_.joint_number[RIGHT_ELBOW_PITCH] = 23;
  arm_joint_infos_.joint_number[RIGHT_ELBOW_ROLL] = 24;
  arm_joint_infos_.joint_number[RIGHT_WRIST_PITCH] = 25;
  arm_joint_infos_.joint_number[RIGHT_WRIST_YAW] = 26;
  arm_joint_infos_.joint_number[WAIST_YAW] = 12;

  arm_joint_limits_.min_angle_rad[LEFT_SHOULDER_PITCH] = -3.14; // -180 deg
  arm_joint_limits_.min_angle_rad[LEFT_SHOULDER_ROLL] = -0.38;  // -22 deg
  arm_joint_limits_.min_angle_rad[LEFT_SHOULDER_YAW] = -3.01; // -172 deg
  arm_joint_limits_.min_angle_rad[LEFT_ELBOW_PITCH] = -2.53; // -145 deg
  arm_joint_limits_.min_angle_rad[LEFT_ELBOW_ROLL] = -2.967; // -170 deg
  arm_joint_limits_.min_angle_rad[LEFT_WRIST_PITCH] = -0.471; // -27 deg
  arm_joint_limits_.min_angle_rad[LEFT_WRIST_YAW] = -1.012; // -58 deg
  arm_joint_limits_.min_angle_rad[RIGHT_SHOULDER_PITCH] = -1.57; // -90 deg
  arm_joint_limits_.min_angle_rad[RIGHT_SHOULDER_ROLL] = -3.4; // -195 deg
  arm_joint_limits_.min_angle_rad[RIGHT_SHOULDER_YAW] = -2.66; // -152 deg
  arm_joint_limits_.min_angle_rad[RIGHT_ELBOW_PITCH] = -1.6; // -91 deg
  arm_joint_limits_.min_angle_rad[RIGHT_ELBOW_ROLL] = -2.967; // -170 deg
  arm_joint_limits_.min_angle_rad[RIGHT_WRIST_PITCH] = -0.471; // -27 deg
  arm_joint_limits_.min_angle_rad[RIGHT_WRIST_YAW] = -1.012; // -58 deg
  arm_joint_limits_.min_angle_rad[WAIST_YAW] = -2.35; // -135 deg

  arm_joint_limits_.max_angle_rad[LEFT_SHOULDER_PITCH] = 1.57; // 90 deg
  arm_joint_limits_.max_angle_rad[LEFT_SHOULDER_ROLL] = 3.4; // 195 deg
  arm_joint_limits_.max_angle_rad[LEFT_SHOULDER_YAW] = 2.66; // 152 deg
  arm_joint_limits_.max_angle_rad[LEFT_ELBOW_PITCH] = 1.6; // 91 deg
  arm_joint_limits_.max_angle_rad[LEFT_ELBOW_ROLL] = 2.967; // 170 deg
  arm_joint_limits_.max_angle_rad[LEFT_WRIST_PITCH] = 0.349; // 20 deg
  arm_joint_limits_.max_angle_rad[LEFT_WRIST_YAW] = 1.012; // 58 deg
  arm_joint_limits_.max_angle_rad[RIGHT_SHOULDER_PITCH] = 3.14; // 180 deg
  arm_joint_limits_.max_angle_rad[RIGHT_SHOULDER_ROLL] = 0.38; // 22 deg
  arm_joint_limits_.max_angle_rad[RIGHT_SHOULDER_YAW] = 3.01; // 172 deg
  arm_joint_limits_.max_angle_rad[RIGHT_ELBOW_PITCH] = 2.53; // 145 deg
  arm_joint_limits_.max_angle_rad[RIGHT_ELBOW_ROLL] = 2.967; // 170 deg
  arm_joint_limits_.max_angle_rad[RIGHT_WRIST_PITCH] = 0.349; // 20 deg
  arm_joint_limits_.max_angle_rad[RIGHT_WRIST_YAW] = 1.012; // 58 deg
  arm_joint_limits_.max_angle_rad[WAIST_YAW] = 2.35; // 135 deg


  this->declare_parameter("controller_freq_hz", 50.0);
  controller_freq_hz_ = this->get_parameter("controller_freq_hz").get_value<double>();

  this->declare_parameter("max_angular_vel_rps", 0.5);
  max_angular_vel_rps_ = this->get_parameter("max_angular_vel_rps").get_value<double>();

  this->declare_parameter("max_angle_delta_rad", 0.01);
  max_angle_delta_rad_ = this->get_parameter("max_angle_delta_rad").get_value<double>();

  this->declare_parameter("min_angle_delta_rad", 0.001);
  min_angle_delta_rad_ = this->get_parameter("min_angle_delta_rad").get_value<double>();

  this->declare_parameter("angle_tolerance_rad", 0.0523599); // 3 deg
  angle_tolerance_rad_ = this->get_parameter("angle_tolerance_rad").get_value<double>();

  this->declare_parameter("control_weight_rate", 0.2);
  control_weight_rate_ = this->get_parameter("control_weight_rate").get_value<double>();

  this->declare_parameter("wave_hand_sec", 5.0);
  wave_hand_sec_ = this->get_parameter("wave_hand_sec").get_value<double>();

  this->declare_parameter("follow_me_sec", 5.0);
  follow_me_sec_ = this->get_parameter("follow_me_sec").get_value<double>();

  this->declare_parameter("its_me_sec", 5.0);
  its_me_sec_ = this->get_parameter("its_me_sec").get_value<double>();

  this->declare_parameter("introduce_sec", 5.0);
  introduce_sec_ = this->get_parameter("introduce_sec").get_value<double>();

  this->declare_parameter("talking_sec", 5.0);
  talking_sec_ = this->get_parameter("talking_sec").get_value<double>();

  this->declare_parameter("stop_control_sec", 2.0);
  stop_control_sec_ = this->get_parameter("stop_control_sec").get_value<double>();


  controller_FSM_ = std::make_shared<ControllerFSM>(
    std::shared_ptr<IArmController>(this),
    controller_freq_hz_,
    max_angular_vel_rps_,
    max_angle_delta_rad_,
    min_angle_delta_rad_,
    angle_tolerance_rad_,
    control_weight_rate_,
    wave_hand_sec_,
    follow_me_sec_,
    its_me_sec_,
    introduce_sec_,
    talking_sec_,
    stop_control_sec_);


  controller_FSM_timer_.SetInterval_ms((1 / controller_freq_hz_) * 1000); // 20 ms
  controller_FSM_timer_.Start();


  pub_arm_joint_infos_ = create_publisher<arm_interfaces::msg::JointInfoList>(
    "arm/joint_infos", 10);

  sub_arm_emergency_stop_ = create_subscription<std_msgs::msg::Bool>(
    "arm/emergency_stop", 10,
    std::bind(&ArmControllerNode::on_subscribed_arm_emergency_stop, this, std::placeholders::_1));

  action_handler_gesture_ = nullptr;

  action_server_gesture_ = rclcpp_action::create_server<arm_interfaces::action::Gesture>(
    shared_from_this(),
    "gesture",
    std::bind(
      &ArmControllerNode::on_received_action_goal_gesture, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&ArmControllerNode::on_received_action_cancel_gesture, this, std::placeholders::_1),
    std::bind(
      &ArmControllerNode::on_received_action_accepted_gesture, this,
      std::placeholders::_1));
}


void ArmControllerNode::on_controller_FSM_timer_elapsed(void * nothing)
{
  controller_FSM_->do_process();
}


bool ArmControllerNode::get_arm_emergency_stop()
{
  // RCLCPP_INFO(get_logger(), "get_arm_emergency_stop() called");

  return arm_emergency_stop_flag_;
}

std::array<double, JOINT_NUMBER> ArmControllerNode::get_arm_joint_infos()
{
  // RCLCPP_INFO(get_logger(), "get_arm_joint_infos() called");
  std::array<double, JOINT_NUMBER> current_angle_array;

  for (int i = 0; i < JOINT_NUMBER; i++) {
    current_angle_array[i] = arm_joint_infos_.joint_angle_rad[i];
  }

  return current_angle_array;
}

bool ArmControllerNode::get_gesture_action_flag()
{
  // RCLCPP_INFO(get_logger(), "get_gesture_action_flag() called");

  return gesture_action_flag_;
}

uint8_t ArmControllerNode::get_gesture_action_type()
{
  // RCLCPP_INFO(get_logger(), "get_gesture_action_type() called");

  return gesture_type_;
}

void ArmControllerNode::set_arm_motor_cmd(
  std::array<double, JOINT_NUMBER> q, std::array<double, JOINT_NUMBER> dq,
  std::array<double, JOINT_NUMBER> kp, std::array<double, JOINT_NUMBER> kd,
  std::array<double, JOINT_NUMBER> tau,
  double control_weight)
{
  // RCLCPP_INFO(get_logger(), "set_arm_joint_angles() called");

  unitree_hg::msg::dds_::LowCmd_ arm_cmd_msg;
  double target_q;
  arm_cmd_msg.motor_cmd().at(27).q(control_weight); // control on/off 모드 설정

  for (int i = 0; i < JOINT_NUMBER; i++) {
    target_q = std::max(q[i], arm_joint_limits_.min_angle_rad[i]);
    target_q = std::min(target_q, arm_joint_limits_.max_angle_rad[i]);

    arm_cmd_msg.motor_cmd().at(arm_joint_infos_.joint_number[i]).q(target_q);
    arm_cmd_msg.motor_cmd().at(arm_joint_infos_.joint_number[i]).dq(dq[i]);
    arm_cmd_msg.motor_cmd().at(arm_joint_infos_.joint_number[i]).kp(kp[i]);
    arm_cmd_msg.motor_cmd().at(arm_joint_infos_.joint_number[i]).kd(kd[i]);
    arm_cmd_msg.motor_cmd().at(arm_joint_infos_.joint_number[i]).tau(tau[i]);
  }


  arm_sdk_publisher_->Write(arm_cmd_msg);
}

void ArmControllerNode::action_gesture_feedback()
{
  // RCLCPP_INFO(get_logger(), "action_gesture_feedback() called");

  if (action_handler_gesture_) {
    auto feedback = std::make_shared<arm_interfaces::action::Gesture::Feedback>();

    feedback->current_joint_angle.resize(JOINT_NUMBER);

    for (int i = 0; i < JOINT_NUMBER; i++) {
      feedback->current_joint_angle[i].joint_idx.number = arm_joint_infos_.joint_number[i];
      feedback->current_joint_angle[i].angle_rad = arm_joint_infos_.joint_angle_rad[i];
    }

    action_handler_gesture_->publish_feedback(feedback);
  }
}

void ArmControllerNode::action_gesture_result(uint8_t result)
{
  // RCLCPP_INFO(get_logger(), "action_gesture_result() called");

  if (action_handler_gesture_) {
    auto result_msg = std::make_shared<arm_interfaces::action::Gesture::Result>();
    result_msg->result = result;

    action_handler_gesture_->succeed(result_msg);
    action_handler_gesture_ = nullptr;
    gesture_action_flag_ = false;
  }
}

bool ArmControllerNode::is_all_topics_ready()
{
  // RCLCPP_INFO(get_logger(), "is_all_topics_ready() called");

  bool ret = true;

  ret &= rt_low_state_flag_;

  rt_low_state_flag_ = false;

  return ret;
}

void ArmControllerNode::reset_topic_flags()
{
  // RCLCPP_INFO(get_logger(), "reset_topic_flags() called");

  arm_emergency_stop_flag_ = false;
  rt_low_state_flag_ = false;
  gesture_action_flag_ = false;

  gesture_type_ = 0;

  action_handler_gesture_ = nullptr;
}

// unitree dds 토픽 콜백 함수
void ArmControllerNode::on_dds_subscribed_rt_lowstate(const void * rt_lowstate_msg)
{
  rt_low_state_flag_ = true;

  auto type_casting = (const unitree_hg::msg::dds_::LowState_ *) rt_lowstate_msg;
  unitree_hg::msg::dds_::LowState_ state_msg;

  memcpy(&state_msg, type_casting, sizeof(unitree_hg::msg::dds_::LowState_));

  for (int i = 0; i < JOINT_NUMBER; i++) {
    arm_joint_infos_.joint_angle_rad[i] =
      state_msg.motor_state().at(arm_joint_infos_.joint_number[i]).q();
  }
}


void ArmControllerNode::on_subscribed_arm_emergency_stop(
  const std_msgs::msg::Bool::SharedPtr arm_emergency_stop_msg)
{
  RCLCPP_INFO(
    get_logger(), "Subscribed /arm/emergency_stop topic: %d",
    arm_emergency_stop_msg->data);

  arm_emergency_stop_flag_ = arm_emergency_stop_msg->data;
}

rclcpp_action::GoalResponse ArmControllerNode::on_received_action_goal_gesture(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const arm_interfaces::action::Gesture::Goal> action_goal_msg)
{
  RCLCPP_INFO(get_logger(), "Received action goal for gesture");

  if (arm_emergency_stop_flag_) {
    RCLCPP_ERROR(get_logger(), "Arm is in emergency stop state");

    return rclcpp_action::GoalResponse::REJECT;
  } else if (gesture_action_flag_) {
    RCLCPP_ERROR(get_logger(), "Gesture action is already in progress");

    return rclcpp_action::GoalResponse::REJECT;
  } else {
    RCLCPP_INFO(get_logger(), "Gesture action accepted");
    RCLCPP_INFO(
      get_logger(), "Gesture action type: %d", action_goal_msg->action);

    gesture_type_ = action_goal_msg->action;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}


rclcpp_action::CancelResponse ArmControllerNode::on_received_action_cancel_gesture(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_interfaces::action::Gesture>>
  action_handler_gesture)
{
  RCLCPP_INFO(get_logger(), "Received action cancel for gesture");

  if (action_handler_gesture_) {
    action_handler_gesture_ = nullptr;
    gesture_action_flag_ = false;

    return rclcpp_action::CancelResponse::ACCEPT;
  } else {
    return rclcpp_action::CancelResponse::REJECT;
  }
}


void ArmControllerNode::on_received_action_accepted_gesture(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_interfaces::action::Gesture>>
  action_handler_gesture)
{
  RCLCPP_INFO(get_logger(), "Received action accepted for gesture");

  gesture_action_flag_ = true;
  action_handler_gesture_ = action_handler_gesture;
}
