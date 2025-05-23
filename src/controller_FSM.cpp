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
  double weight_rate)
: arm_controller_node_(arm_controller_node),
  controller_freq_hz_(controller_freq_hz),
  controller_freq_sec_(1.0 / controller_freq_hz_),
  max_angular_vel_rps_(max_angular_vel_rps),
  max_angle_delta_rad_(max_angle_delta_rad),
  min_angle_delta_rad_(min_angle_delta_rad),
  angle_tolerance_rad_(angle_tolerance_rad),
  weight_rate_(weight_rate)
{
  initialize();
  initialize_FSM();
}


void ControllerFSM::initialize()
{
  wave_hand_phase_limit_ = 5 * controller_freq_hz_; // 5 sec
  stop_control_phase_limit_ = 2 * controller_freq_hz_; // 2 sec
}

void ControllerFSM::initialize_FSM()
{
  auto initial_state = fsm_.CreateState("0. initial_state");

  auto waiting_topics_state = fsm_.CreateState("1. waiting_topics_state");
  auto waiting_gesture_action_state = fsm_.CreateState("2. waiting_gesture_action_state");
  auto wave_hand_state_1 = fsm_.CreateState("3_1. wave_hand_state_1");
  auto wave_hand_state_2 = fsm_.CreateState("3_2. wave_hand_state_2");
  auto wave_hand_state_3 = fsm_.CreateState("3_3. wave_hand_state_3");
  auto emergency_stop_state = fsm_.CreateState("99. emergency_stop_state");
  auto finish_state = fsm_.CreateState("100. finish_state");

  fsm_.SetInitialState(initial_state);

  initial_state->AddNextState(waiting_topics_state);
  waiting_topics_state->AddNextState(waiting_gesture_action_state);
  waiting_gesture_action_state->AddNextState(wave_hand_state_1);
  waiting_gesture_action_state->AddNextState(emergency_stop_state);
  wave_hand_state_1->AddNextState(wave_hand_state_2);
  wave_hand_state_1->AddNextState(emergency_stop_state);
  wave_hand_state_2->AddNextState(wave_hand_state_3);
  wave_hand_state_2->AddNextState(emergency_stop_state);
  wave_hand_state_3->AddNextState(finish_state);
  wave_hand_state_3->AddNextState(emergency_stop_state);
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

      // for test
      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();

      // for (int i = 0; i < JOINT_NUMBER; i++) {
      //   std::cout << "Joint " << i << ": " << current_pos_array_[i] << std::endl;
      // }

      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "1. waiting_topics_state finish condition check" << std::endl;

      return arm_controller_node_->is_all_topics_ready();
      // return true;
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

      // return arm_controller_node_->get_gesture_action_flag();
      return true;
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
        return false;
      }
    },
    [&]()
    {
      // initial execution of state
      std::cout << "3_1. wave_hand_state_1 initialize" << std::endl;

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();
      target_pos_array_ = current_pos_array_;

      // for (int i = 0; i < JOINT_NUMBER; i++) {
      //   std::cout << "Joint " << i << ": " << current_pos_array_[i] * 180 / Pi <<
      //     std::endl;
      // }

      weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_1. wave_hand_state_1 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      // double delta_weight = weight_rate_ * controller_freq_sec_;
      // weight_ += delta_weight;
      // weight_ = std::clamp(weight_, 0.0, 1.0);

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();


      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = wave_hand_pos_array_[i] - current_pos_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit) *
        std::fabs(joint_angles_diff[i] / max_angle_diff);

        // angle_delta_per_control = (std::fabs(angle_delta_per_control) >= min_angle_delta_rad_) ?
        // angle_delta_per_control :
        // SIGN(angle_delta_per_control) * min_angle_delta_rad_;

        target_pos_array_[i] += angle_delta_per_control;

        // std::cout << "Joint target" << i << ": " << target_pos_array_[i] << std::endl;
        // std::cout << "wave pos " << i << ": " << wave_hand_pos_array_[i] * 180 / Pi <<
        //   std::endl;

        // std::cout << "current pos " << i << ": " << current_pos_array_[i] * 180 / Pi <<
        //   std::endl;
        // std::cout << "angle delta per control " << i << ": " <<
        //   angle_delta_per_control * 180 / Pi <<
        //   std::endl;
        // std::cout << "target pos " << i << ": " << target_pos_array_[i] * 180 / Pi <<
        //   std::endl;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_pos_array_, std::array<double, JOINT_NUMBER>{0.0f}, joint_kp_array_,
        joint_kd_array_, std::array<double, JOINT_NUMBER>{0.0f}, weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_1. wave_hand_state_1 finish condition check" << std::endl;

      bool ret = true;


      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::abs(current_pos_array_[i] - wave_hand_pos_array_[i]) > angle_tolerance_rad_) {
          ret = false;
          break;
        }
      }

      ret = false;

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

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();
      target_pos_array_ = current_pos_array_;

      wave_hand_phase_ = 0;

      weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_2. wave_hand_state_2 periodic action" << std::endl;

      double angle_delta_per_control;
      double phase = wave_hand_phase_ / wave_hand_phase_limit_;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      // double delta_weight = weight_rate_ * controller_freq_sec_;
      // weight_ += delta_weight;
      // weight_ = std::clamp(weight_, 0.0, 1.0);

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();

      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      wave_hand_pos_array_[RIGHT_SHOULDER_YAW] = deg_30 * std::sin(4 * Pi * phase);


      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          wave_hand_pos_array_[i] - current_pos_array_[i],
          -angle_delta_limit, angle_delta_limit);

        // angle_delta_per_control = (std::fabs(angle_delta_per_control) >= min_angle_delta_rad_) ?
        // angle_delta_per_control :
        // SIGN(angle_delta_per_control) * min_angle_delta_rad_;

        target_pos_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_pos_array_, std::array<double, JOINT_NUMBER>{0.0f}, joint_kp_array_,
        joint_kd_array_, std::array<double, JOINT_NUMBER>{0.0f}, weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_2. wave_hand_state_2 finish condition check" << std::endl;

      if (wave_hand_phase_ >= wave_hand_phase_limit_) {
        wave_hand_pos_array_[RIGHT_SHOULDER_YAW] = 0;
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

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();
      target_pos_array_ = current_pos_array_;

      weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3_3. wave_hand_state_3 periodic action" << std::endl;

      std::array<double, JOINT_NUMBER> joint_angles_diff;
      double max_angle_diff;
      double angle_delta_per_control;

      double angle_delta_limit = max_angular_vel_rps_ * controller_freq_sec_;

      // double delta_weight = weight_rate_ * controller_freq_sec_;
      // weight_ += delta_weight;
      // weight_ = std::clamp(weight_, 0.0, 1.0);

      current_pos_array_ = arm_controller_node_->get_arm_joint_infos();


      angle_delta_limit = std::clamp(angle_delta_limit, 0.0, max_angle_delta_rad_);

      for (int i = 0; i < JOINT_NUMBER; i++) {
        joint_angles_diff[i] = init_pos_array_[i] - current_pos_array_[i];
      }

      for (double d: joint_angles_diff) {
        max_angle_diff = std::max(max_angle_diff, std::fabs(d));
      }

      for (int i = 0; i < JOINT_NUMBER; i++) {
        angle_delta_per_control = std::clamp(
          joint_angles_diff[i], -angle_delta_limit,
          angle_delta_limit) *
        std::fabs(joint_angles_diff[i] / max_angle_diff);

        // angle_delta_per_control = (std::fabs(angle_delta_per_control) >= min_angle_delta_rad_) ?
        // angle_delta_per_control :
        // SIGN(angle_delta_per_control) * min_angle_delta_rad_;

        target_pos_array_[i] += angle_delta_per_control;
      }

      arm_controller_node_->set_arm_motor_cmd(
        target_pos_array_, std::array<double, JOINT_NUMBER>{0.0f}, joint_kp_array_,
        joint_kd_array_, std::array<double, JOINT_NUMBER>{0.0f}, weight_);


      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3_3. wave_hand_state_3 finish condition check" << std::endl;

      bool ret = true;


      for (int i = 0; i < JOINT_NUMBER; i++) {
        if (std::abs(current_pos_array_[i] - init_pos_array_[i]) > angle_tolerance_rad_) {
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
      weight_ = 1.0;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "99. emergency_stop_state periodic action" << std::endl;

      double delta_weight = weight_rate_ * controller_freq_sec_;
      weight_ -= delta_weight;
      weight_ = std::clamp(weight_, 0.0, 1.0);

      arm_controller_node_->set_arm_motor_cmd(
        std::array<double, JOINT_NUMBER>{0.0f}, std::array<double, JOINT_NUMBER>{0.0f},
        std::array<double, JOINT_NUMBER>{0.0f}, std::array<double, JOINT_NUMBER>{0.0f},
        std::array<double, JOINT_NUMBER>{0.0f}, weight_);

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
      // std::cout << "100. finish_state condition check" << std::endl;
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
