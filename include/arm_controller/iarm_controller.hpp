#ifndef ARM_CONTROLLER__IARM_CONTROLLER_HPP_
#define ARM_CONTROLLER__IARM_CONTROLLER_HPP_

#include "arm_controller/definitions.hpp"

#include <array>


class IArmController
{
public:
  virtual bool get_arm_emergency_stop() = 0;
  virtual std::array<double, JOINT_NUMBER> get_arm_joint_infos() = 0;
  virtual bool get_gesture_action_flag() = 0;
  virtual uint8_t get_gesture_action_type() = 0;

  virtual void set_arm_motor_cmd(
    std::array<double, JOINT_NUMBER> q, std::array<double, JOINT_NUMBER> dq,
    std::array<double, JOINT_NUMBER> kp, std::array<double, JOINT_NUMBER> kd,
    std::array<double, JOINT_NUMBER> tau,
    double control_weight = 1.0) = 0;

  virtual bool is_all_topics_ready() = 0;
  virtual void reset_topic_flags() = 0;
};


#endif // ARM_CONTROLLER__IARM_CONTROLLER_HPP_
