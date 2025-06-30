#ifndef ARM_CONTROLLER__CONTROLLER_FSM_HPP_
#define ARM_CONTROLLER__CONTROLLER_FSM_HPP_


#include "arm_controller/definitions.hpp"
#include "arm_controller/iarm_controller.hpp"

#include <cmath>

#include "clobot_platform_libs/finite_state_machine/machine.hpp"


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

  double control_weight_rate_;
  double control_weight_ = 0.0;
// control parameters

  bool emergency_stop_flag_ = false;

  double wave_hand_sec_;
  double follow_me_sec_;
  double its_me_sec_;
  double introduce_sec_;
  double talking_sec_;
  double stop_control_sec_;

  int wave_hand_phase_ = 0;
  int wave_hand_phase_limit_;

  int follow_me_phase_ = 0;
  int follow_me_phase_limit_;

  int its_me_phase_ = 0;
  int its_me_phase_limit_;

  int introduce_phase_ = 0;
  int introduce_phase_limit_;

  int talking_phase_ = 0;
  int talking_phase_limit_;

  int stop_control_phase_ = 0;
  int stop_control_phase_limit_;


  std::array<double, JOINT_NUMBER> current_angle_array_ = {0};
  std::array<double, JOINT_NUMBER> target_angle_array_ = {0};

  std::array<double, JOINT_NUMBER> joint_kp_array_ = {120, 120, 80, 50, 50, 50, 50,
    120, 120, 80, 50, 50, 50, 50,
    200};
  std::array<double, JOINT_NUMBER> joint_kd_array_ = {2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0,
    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0,
    2.0};

  std::array<double, JOINT_NUMBER> init_angle_array_ = {deg_10, deg_15, 0.f, 0, 0, 0, 0,
    deg_10, -deg_15, 0.f, 0, 0, 0, 0,
    0.f};

  std::array<double, JOINT_NUMBER> wave_hand_angle_array_ = {Pi_4, deg_20, 0, Pi_4, 0, 0, 0,
    -Pi_4, -deg_20, 0, -Pi_4, -Pi_2, 0, 0,
    0};

  std::array<double, JOINT_NUMBER> follow_me_angle_array_ = {Pi_4, deg_20, 0, Pi_4, 0, 0, 0,
    -deg_30, 0, deg_10, 0, Pi_2, 0, 0,
    0};

  std::array<double, JOINT_NUMBER> its_me_angle_array_ = {Pi_4, deg_20, 0, Pi_4, 0, 0, 0,
    -Pi_4, -deg_20, 0, -Pi_4, -Pi_2, 0, 0,
    0};

  std::array<double, JOINT_NUMBER> introduce_angle_array_ = {-deg_10, deg_30, Pi_4, deg_60, 0, 0, 0,
    deg_10, -deg_15, 0.f, deg_60, 0, 0, 0,
    0.f};

  std::array<double, JOINT_NUMBER> talking_angle_array_ = {deg_30, deg_20, 0, Pi_4, 0, 0, 0,
    -deg_10, -deg_30, 0, -deg_30, 0, 0, 0,
    0};

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
    , double control_weight_rate
    , double wave_hand_sec
    , double follow_me_sec
    , double its_me_sec
    , double introduce_sec
    , double talking_sec
    , double stop_control_sec);

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
