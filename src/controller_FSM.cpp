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


#include "arm_controller/controller_FSM.hpp"


ControllerFSM::ControllerFSM(
  std::shared_ptr<IArmController> arm_controller_node,
  double controller_freq_hz,
  double max_angular_vel_rps,
  double max_angle_delta_rad,
  double min_angle_delta_rad,
  double angle_tolerance_rad,
  double control_weight_rate,
  double wave_hand_sec,
  double follow_me_sec,
  double its_me_sec,
  double introduce_sec,
  double talking_sec,
  double stop_control_sec)
: arm_controller_node_(arm_controller_node),
  controller_freq_hz_(controller_freq_hz),
  controller_freq_sec_(1.0 / controller_freq_hz_),
  max_angular_vel_rps_(max_angular_vel_rps),
  max_angle_delta_rad_(max_angle_delta_rad),
  min_angle_delta_rad_(min_angle_delta_rad),
  angle_tolerance_rad_(angle_tolerance_rad),
  control_weight_rate_(control_weight_rate),
  wave_hand_sec_(wave_hand_sec),
  follow_me_sec_(follow_me_sec),
  its_me_sec_(its_me_sec),
  introduce_sec_(introduce_sec),
  talking_sec_(talking_sec),
  stop_control_sec_(stop_control_sec)
{
  initialize();
  initialize_FSM();
}


void ControllerFSM::initialize()
{
  wave_hand_phase_limit_ = wave_hand_sec_ * controller_freq_hz_; // 5 sec
  follow_me_phase_limit_ = follow_me_sec_ * controller_freq_hz_; // 5 sec
  its_me_phase_limit_ = its_me_sec_ * controller_freq_hz_; // 5 sec
  introduce_phase_limit_ = introduce_sec_ * controller_freq_hz_; // 5 sec
  talking_phase_limit_ = talking_sec_ * controller_freq_hz_; // 5 sec
  stop_control_phase_limit_ = stop_control_sec_ * controller_freq_hz_; // 2 sec
}

void ControllerFSM::initialize_FSM()
{
  auto initial_state = fsm_.CreateState("0. initial_state");

  auto waiting_topics_state = fsm_.CreateState("1. waiting_topics_state");
  auto waiting_gesture_action_state = fsm_.CreateState("2. waiting_gesture_action_state");
  auto wave_hand_state_1 = fsm_.CreateState("3_1. wave_hand_state_1");
  auto wave_hand_state_2 = fsm_.CreateState("3_2. wave_hand_state_2");
  auto wave_hand_state_3 = fsm_.CreateState("3_3. wave_hand_state_3");
  auto follow_me_state_1 = fsm_.CreateState("4_1. follow_me_state_1");
  auto follow_me_state_2 = fsm_.CreateState("4_2. follow_me_state_2");
  auto follow_me_state_3 = fsm_.CreateState("4_3. follow_me_state_3");
  auto its_me_state_1 = fsm_.CreateState("5_1. its_me_state_1");
  auto its_me_state_2 = fsm_.CreateState("5_2. its_me_state_2");
  auto its_me_state_3 = fsm_.CreateState("5_3. its_me_state_3");
  auto introduce_state_1 = fsm_.CreateState("6_1. introduce_state_1");
  auto introduce_state_2 = fsm_.CreateState("6_2. introduce_state_2");
  auto introduce_state_3 = fsm_.CreateState("6_3. introduce_state_3");
  auto talking_state_1 = fsm_.CreateState("7_1. talking_state_1");
  auto talking_state_2 = fsm_.CreateState("7_2. talking_state_2");
  auto talking_state_3 = fsm_.CreateState("7_3. talking_state_3");
  auto emergency_stop_state = fsm_.CreateState("99. emergency_stop_state");
  auto finish_state = fsm_.CreateState("100. finish_state");

  fsm_.SetInitialState(initial_state);

  initial_state->AddNextState(waiting_topics_state);
  waiting_topics_state->AddNextState(waiting_gesture_action_state);
  waiting_gesture_action_state->AddNextState(wave_hand_state_1);
  waiting_gesture_action_state->AddNextState(follow_me_state_1);
  waiting_gesture_action_state->AddNextState(its_me_state_1);
  waiting_gesture_action_state->AddNextState(introduce_state_1);
  waiting_gesture_action_state->AddNextState(talking_state_1);
  waiting_gesture_action_state->AddNextState(emergency_stop_state);
  wave_hand_state_1->AddNextState(wave_hand_state_2);
  wave_hand_state_1->AddNextState(emergency_stop_state);
  wave_hand_state_2->AddNextState(wave_hand_state_3);
  wave_hand_state_2->AddNextState(emergency_stop_state);
  wave_hand_state_3->AddNextState(finish_state);
  wave_hand_state_3->AddNextState(emergency_stop_state);
  follow_me_state_1->AddNextState(follow_me_state_2);
  follow_me_state_1->AddNextState(emergency_stop_state);
  follow_me_state_2->AddNextState(follow_me_state_3);
  follow_me_state_2->AddNextState(emergency_stop_state);
  follow_me_state_3->AddNextState(finish_state);
  follow_me_state_3->AddNextState(emergency_stop_state);
  its_me_state_1->AddNextState(its_me_state_2);
  its_me_state_1->AddNextState(emergency_stop_state);
  its_me_state_2->AddNextState(its_me_state_3);
  its_me_state_2->AddNextState(emergency_stop_state);
  its_me_state_3->AddNextState(finish_state);
  its_me_state_3->AddNextState(emergency_stop_state);
  introduce_state_1->AddNextState(introduce_state_2);
  introduce_state_1->AddNextState(emergency_stop_state);
  introduce_state_2->AddNextState(introduce_state_3);
  introduce_state_2->AddNextState(emergency_stop_state);
  introduce_state_3->AddNextState(finish_state);
  introduce_state_3->AddNextState(emergency_stop_state);
  talking_state_1->AddNextState(talking_state_2);
  talking_state_1->AddNextState(emergency_stop_state);
  talking_state_2->AddNextState(talking_state_3);
  talking_state_2->AddNextState(emergency_stop_state);
  talking_state_3->AddNextState(finish_state);
  talking_state_3->AddNextState(emergency_stop_state);
  emergency_stop_state->AddNextState(finish_state);
  finish_state->AddNextState(initial_state);


  // 0. initial_state
  initial_state->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "0. initial_state condition check" << std::endl;
      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "0. initial_state initialize" << std::endl;
    },
    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "0. initial_state periodic action" << std::endl;
      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "0. initial_state finish condition check" << std::endl;
      return true;
    });


  // 1. waiting_topics_state
  waiting_topics_state->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "1. waiting_topics_state condition check" << std::endl;
      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "1. waiting_topics_state initialize" << std::endl;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "1. waiting_topics_state periodic action" << std::endl;

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "1. waiting_topics_state finish condition check" << std::endl;

      return arm_controller_node_->is_all_topics_ready();
    });

  // 2. waiting_gesture_action_state
  waiting_gesture_action_state->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "2. waiting_gesture_action_state condition check" << std::endl;
      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "2. waiting_gesture_action_state initialize" << std::endl;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "2. waiting_gesture_action_state periodic action" << std::endl;
      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "2. waiting_gesture_action_state finish condition check" << std::endl;

      return arm_controller_node_->get_gesture_action_flag();
    });

  // 3_1. wave_hand_state_1
  wave_hand_state_1->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "3_1. wave_hand_state_1 condition check" << std::endl;

      if (arm_controller_node_->get_gesture_action_type() == GESTURE_WAVE_HAND) {
        std::cout << "Gesture action type: " << GESTURE_WAVE_HAND << std::endl;
        return true;
      } else {
        std::cout << "Gesture action type is not GESTURE_WAVE_HAND" << std::endl;
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "3_1. wave_hand_state_1 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_1. wave_hand_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = wave_hand_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_1. wave_hand_state_1 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - wave_hand_angle_array_[i]) >=
        angle_tolerance_rad_)
        {
          ret = false;
          break;
        }
      }

      return ret;
    });


  // 3_2. wave_hand_state_2
  wave_hand_state_2->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "3_2. wave_hand_state_2 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "3_2. wave_hand_state_2 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      wave_hand_phase_ = 0;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_2. wave_hand_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = (double) (wave_hand_phase_) / wave_hand_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      std::array<double, JOINT_NUMBER> current_wave_hand_angle_array = current_angle_array_;
      current_wave_hand_angle_array[RIGHT_SHOULDER_YAW] = deg_30 * std::sin(4 * Pi * phase);

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          current_wave_hand_angle_array[i] - current_angle_array_[i],
          -angle_delta_limit, angle_delta_limit);

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_2. wave_hand_state_2 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();


      if (wave_hand_phase_ >= wave_hand_phase_limit_) {
        return true;
      } else {
        wave_hand_phase_++;
        return false;
      }
    });


  // 3_3. wave_hand_state_3
  wave_hand_state_3->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "3_3. wave_hand_state_3 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "3_3. wave_hand_state_3 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_3. wave_hand_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_3. wave_hand_state_3 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - init_angle_array_[i]) >= angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      return ret;
    });


  // 4_1. follow_me_state_1
  follow_me_state_1->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "4_1. follow_me_state_1 condition check" << std::endl;

      if (arm_controller_node_->get_gesture_action_type() == GESTURE_FOLLOW_ME) {
        std::cout << "Gesture action type: " << GESTURE_FOLLOW_ME << std::endl;
        return true;
      } else {
        std::cout << "Gesture action type is not GESTURE_FOLLOW_ME" << std::endl;
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "4_1. follow_me_state_1 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "4_1. follow_me_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = follow_me_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "4_1. follow_me_state_1 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - follow_me_angle_array_[i]) >=
        angle_tolerance_rad_)
        {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 4_2. follow_me_state_2
  follow_me_state_2->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "4_2. follow_me_state_2 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "4_2. follow_me_state_2 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      follow_me_phase_ = 0;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "4_2. follow_me_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = (double) (follow_me_phase_) / follow_me_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      std::array<double, JOINT_NUMBER> current_follow_me_angle_array = current_angle_array_;
      current_follow_me_angle_array[RIGHT_ELBOW_PITCH] = -(deg_20 *
      std::sin(4 * Pi * (phase - 1 / 8)) + deg_20);

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          current_follow_me_angle_array[i] - current_angle_array_[i],
          -angle_delta_limit, angle_delta_limit);

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "4_2. follow_me_state_2 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();


      if (follow_me_phase_ >= follow_me_phase_limit_) {
        return true;
      } else {
        follow_me_phase_++;
        return false;
      }
    });

  // 4_3. follow_me_state_3
  follow_me_state_3->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "4_3. follow_me_state_3 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "4_3. follow_me_state_3 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "4_3. follow_me_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "4_3. follow_me_state_3 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - init_angle_array_[i]) >= angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 5_1. its_me_state_1
  its_me_state_1->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "5_1. its_me_state_1 condition check" << std::endl;

      if (arm_controller_node_->get_gesture_action_type() == GESTURE_ITS_ME) {
        std::cout << "Gesture action type: " << GESTURE_ITS_ME << std::endl;
        return true;
      } else {
        std::cout << "Gesture action type is not GESTURE_ITS_ME" << std::endl;
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "5_1. its_me_state_1 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "5_1. its_me_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = its_me_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "5_1. its_me_state_1 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - its_me_angle_array_[i]) >=
        angle_tolerance_rad_)
        {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 5_2. its_me_state_2
  its_me_state_2->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "5_2. its_me_state_2 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "5_2. its_me_state_2 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      its_me_phase_ = 0;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "5_2. its_me_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = (double) (its_me_phase_) / its_me_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      std::array<double, JOINT_NUMBER> current_its_me_angle_array = current_angle_array_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          current_its_me_angle_array[i] - current_angle_array_[i],
          -angle_delta_limit, angle_delta_limit);

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "5_2. its_me_state_2 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();


      if (its_me_phase_ >= its_me_phase_limit_) {
        return true;
      } else {
        its_me_phase_++;
        return false;
      }
    });

  // 5_3. its_me_state_3
  its_me_state_3->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "5_3. its_me_state_3 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "5_3. its_me_state_3 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "5_3. its_me_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "5_3. its_me_state_3 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - init_angle_array_[i]) >= angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 6_1. introduce_state_1
  introduce_state_1->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "6_1. introduce_state_1 condition check" << std::endl;

      if (arm_controller_node_->get_gesture_action_type() == GESTURE_INTRODUCE) {
        std::cout << "Gesture action type: " << GESTURE_INTRODUCE << std::endl;
        return true;
      } else {
        std::cout << "Gesture action type is not GESTURE_INTRODUCE" << std::endl;
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "6_1. introduce_state_1 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "6_1. introduce_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = introduce_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "6_1. introduce_state_1 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - introduce_angle_array_[i]) >=
        angle_tolerance_rad_)
        {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 6_2. introduce_state_2
  introduce_state_2->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "6_2. introduce_state_2 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "6_2. introduce_state_2 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      introduce_phase_ = 0;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "6_2. introduce_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = (double) (introduce_phase_) / introduce_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      std::array<double, JOINT_NUMBER> current_introduce_angle_array = current_angle_array_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          current_introduce_angle_array[i] - current_angle_array_[i],
          -angle_delta_limit, angle_delta_limit);

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "6_2. introduce_state_2 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();


      if (introduce_phase_ >= introduce_phase_limit_) {
        return true;
      } else {
        introduce_phase_++;
        return false;
      }
    });

  // 6_3. introduce_state_3
  introduce_state_3->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "6_3. introduce_state_3 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "6_3. introduce_state_3 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "6_3. introduce_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "6_3. introduce_state_3 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - init_angle_array_[i]) >= angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 7_1. talking_state_1
  talking_state_1->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "7_1. talking_state_1 condition check" << std::endl;

      if (arm_controller_node_->get_gesture_action_type() == GESTURE_TALKING) {
        std::cout << "Gesture action type: " << GESTURE_TALKING << std::endl;
        return true;
      } else {
        std::cout << "Gesture action type is not GESTURE_TALKING" << std::endl;
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "7_1. talking_state_1 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "7_1. talking_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = talking_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "7_1. talking_state_1 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - talking_angle_array_[i]) >=
        angle_tolerance_rad_)
        {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 7_2. talking_state_2
  talking_state_2->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "7_2. talking_state_2 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "7_2. talking_state_2 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      talking_phase_ = 0;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "7_2. talking_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = (double) (talking_phase_) / talking_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      std::array<double, JOINT_NUMBER> current_talking_angle_array = current_angle_array_;
      // current_talking_angle_array[RIGHT_SHOULDER_PITCH] = -deg_20 * std::sin(8 * Pi * phase);

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          current_talking_angle_array[i] - current_angle_array_[i],
          -angle_delta_limit, angle_delta_limit);

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "7_2. talking_state_2 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();


      if (talking_phase_ >= talking_phase_limit_) {
        return true;
      } else {
        talking_phase_++;
        return false;
      }
    });

  // 7_3. talking_state_3
  talking_state_3->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "7_3. talking_state_3 condition check" << std::endl;

      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "7_3. talking_state_3 initialize" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();
      target_angle_array_ = current_angle_array_;

      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "7_3. talking_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff = 0.0;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_angle_array_[i] - current_angle_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit);

        if (max_angle_diff != 0.0) {
          angle_delta_per_control *= std::fabs(joint_angles_diff[i] / max_angle_diff);
        }

        target_angle_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "7_3. talking_state_3 finish condition check" << std::endl;

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      bool ret = true;

      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::fabs(current_angle_array_[i] - init_angle_array_[i]) >= angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      return ret;
    });

  // 99. emergency_stop_state
  emergency_stop_state->SetState(
    false, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      // std::cout << "99. emergency_stop_state condition check" << std::endl;
      if (arm_controller_node_->get_arm_emergency_stop()) {
        return true;
      } else {
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "99. emergency_stop_state initialize" << std::endl;

      stop_control_phase_ = 0;
      control_weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "99. emergency_stop_state periodic action" << std::endl;

      double delta_control_weight = control_weight_rate_ * controller_freq_sec_;
      control_weight_ -= delta_control_weight;
      control_weight_ = std::clamp(control_weight_, 0.0, 1.0);

      current_angle_array_ = arm_controller_node_->get_arm_joint_infos();

      arm_controller_node_->set_arm_motor_cmd(
        current_angle_array_, std::array<double, JOINT_NUMBER>{0.0f},
        joint_kp_array_, joint_kd_array_,
        std::array<double, JOINT_NUMBER>{0.0f},
        control_weight_);

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "99. emergency_stop_state finish condition check" << std::endl;

      if (stop_control_phase_ >= stop_control_phase_limit_) {
        return true;
      } else {
        stop_control_phase_++;
        return false;
      }
    });

  // 100. finish_state
  finish_state->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      std::cout << "100. finish_state condition check" << std::endl;
      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "100. finish_state initialize" << std::endl;

      arm_controller_node_->reset_topic_flags();
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "100. finish_state periodic action" << std::endl;

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "100. finish_state finish condition check" << std::endl;

      return true;
    });
}

void ControllerFSM::do_process()
{
  fsm_.ProcessState();
}
