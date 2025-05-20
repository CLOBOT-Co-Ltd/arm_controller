#ifndef ARM_CONTROLLER__CONTROLLER_FSM_HPP_
#define ARM_CONTROLLER__CONTROLLER_FSM_HPP_

#include "arm_controller/definitions.hpp"
#include "arm_controller/iarm_controller.hpp"

#include <cmath>

#include "clobot_platform_libs/finite_state_machine/machine.hpp"


class ControllerFSM
{
private:
  double controller_freq_hz_; // hz
  double controller_freq_sec_; // sec

  double max_angular_vel_rps_; // 모터 최대 각속도 (rad/sec)

  double max_angle_delta_rad_; // 제어주기별 모터 최대 각도 변화량 (rad)
  double min_angle_delta_rad_; // 제어주기별 모터 최소 각도 변화량 (rad)

  std::shared_ptr<IArmController> arm_controller_node_;
  clobot_platform_libs::finite_state_machine::Machine fsm_;

public:
  ControllerFSM(
    std::shared_ptr<IArmController> arm_controller_node
    , double controller_freq_hz
    , double max_angular_vel_rps
    , double max_angle_delta_rad
    , double min_angle_delta_rad);

  // ~ControllerFSM();

// 각 필드 값 초기화
  void initialize();

// state 순서 연결 & FSM 에 등록
  void initialize_FSM();

public:
// FSM 가동 메소드
  void do_process();
};


#endif // ARM_CONTROLLER__CONTROLLER_FSM_HPP_
