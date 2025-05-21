#ifndef ARM_CONTROLLER__CONTROLLER_FSM_HPP_
#define ARM_CONTROLLER__CONTROLLER_FSM_HPP_

#define SIGN(val) ((val >= 0.0) ? 1.0 : -1.0)


#include "arm_controller/definitions.hpp"
#include "arm_controller/iarm_controller.hpp"

#include <cmath>

#include "clobot_platform_libs/finite_state_machine/machine.hpp"


const double Pi = 3.141592654;
const double Pi_2 = 1.57079632;
const double Pi_4 = 0.78539816;
const double deg_30 = 0.523599;
const double deg_20 = 0.349066;

const uint8_t GESTURE_WAVE_HAND = 0;

class ControllerFSM
{
private:
// control parameters
  double controller_freq_hz_; // hz
  double controller_freq_sec_; // sec

  double max_angular_vel_rps_; // 모터 최대 각속도 (rad/sec)

  double max_angle_delta_rad_; // 제어주기별 모터 최대 각도 변화량 (rad)
  double min_angle_delta_rad_; // 제어주기별 모터 최소 각도 변화량 (rad)

  double angle_tolerance_rad_; // 모터 각도 허용 오차 (rad)

  double weight_rate_;
  double weight_ = 0.0;
// control parameters

  std::array<double, JOINT_NUMBER> current_pos_array_ = {0};
  std::array<double, JOINT_NUMBER> target_pos_array_ = {0};

  std::array<double, JOINT_NUMBER> joint_kp_array_ = {120, 120, 80, 50, 50, 50, 50,
    120, 120, 80, 50, 50, 50, 50,
    200};
  std::array<double, JOINT_NUMBER> joint_kd_array_ = {2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0,
    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0,
    2.0};


  // std::array<double, JOINT_NUMBER> joint_kp_array_ = {10, 10, 10, 10, 10, 10, 10,
  //   10, 10, 10, 10, 10, 10, 10,
  //   10};
  // std::array<double, JOINT_NUMBER> joint_kd_array_ = {10, 10, 10, 10, 10, 10, 10,
  //   10, 10, 10, 10, 10, 10, 10,
  //   10};

  std::array<double, JOINT_NUMBER> init_pos_array_ = {0.f, 0.3, 0.f, 0, 0, 0, 0,
    0.f, -0.3, 0.f, 0, 0, 0, 0,
    0.f};

  std::array<double, JOINT_NUMBER> wave_hand_pos_array_ = {Pi_2 / 2, deg_20, 0, Pi_2 / 2, 0, 0, 0,
    -Pi_2 / 2, -deg_20, 0, -Pi_2 / 2, -Pi_2, 0, 0,
    0};

  int wave_hand_phase_ = 0;
  int wave_hand_phase_limit_;

  int stop_control_phase_ = 0;
  int stop_control_phase_limit_;


  std::shared_ptr<IArmController> arm_controller_node_;
  clobot_platform_libs::finite_state_machine::Machine fsm_;

public:
  ControllerFSM(
    std::shared_ptr<IArmController> arm_controller_node
    , double controller_freq_hz
    , double max_angular_vel_rps
    , double max_angle_delta_rad
    , double min_angle_delta_rad
    , double angle_tolerance_rad
    , double weight_rate);

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
