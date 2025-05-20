#ifndef ARM_CONTROLLER__IARM_CONTROLLER_HPP_
#define ARM_CONTROLLER__IARM_CONTROLLER_HPP_

#include "arm_controller/definitions.hpp"

class IArmController
{
public:
  virtual bool get_arm_emergency_stop() = 0;
  virtual JOINT_INFOS get_arm_joint_infos() = 0;
  virtual bool get_gesture_action_flag() = 0;

  virtual void set_arm_joint_angles(float * joint_angles_rad_cmd) = 0;

  virtual bool is_all_topics_ready() = 0;
  virtual void reset_topic_flags() = 0;
};


#endif // ARM_CONTROLLER__IARM_CONTROLLER_HPP_
