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
  double min_angle_delta_rad)
: arm_controller_node_(arm_controller_node),
  controller_freq_hz_(controller_freq_hz),
  controller_freq_sec_(1.0 / controller_freq_hz_),
  max_angular_vel_rps_(max_angular_vel_rps),
  max_angle_delta_rad_(max_angle_delta_rad),
  min_angle_delta_rad_(min_angle_delta_rad)
{
  initialize();
  initialize_FSM();
}


void ControllerFSM::initialize()
{
}

void ControllerFSM::initialize_FSM()
{
  auto initial_state = fsm_.CreateState("0. initial_state");

  auto waiting_topics_state = fsm_.CreateState("1. waiting_topics_state");
  auto waiting_gesture_action_state = fsm_.CreateState("2. waiting_gesture_action_state");
  auto gesture_action_state = fsm_.CreateState("3. gesture_action_state");
  auto emergency_stop_state = fsm_.CreateState("99. emergency_stop_state");
  auto finish_state = fsm_.CreateState("100. finish_state");

  fsm_.SetInitialState(initial_state);

  initial_state->AddNextState(waiting_topics_state);
  waiting_topics_state->AddNextState(waiting_gesture_action_state);
  waiting_gesture_action_state->AddNextState(gesture_action_state);
  waiting_gesture_action_state->AddNextState(emergency_stop_state);
  gesture_action_state->AddNextState(finish_state);
  gesture_action_state->AddNextState(emergency_stop_state);
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
      // std::cout << "1. waiting_topics_state condition check" << std::endl;
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
      // std::cout << "2. waiting_gesture_action_state condition check" << std::endl;
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

  // 3. gesture_action_state
  gesture_action_state->SetState(
    true, // previous state complete check or not

    [&]() -> bool
    {
      // condition of state entry
      // std::cout << "3. gesture_action_state condition check" << std::endl;
      return true;
    },
    [&]()
    {
      // initial execution of state
      std::cout << "3. gesture_action_state initialize" << std::endl;
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "3. gesture_action_state periodic action" << std::endl;
      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "3. gesture_action_state finish condition check" << std::endl;

      return true;
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
    },

    [&]() -> bool
    {
      // periodic action of state
      // std::cout << "99. emergency_stop_state periodic action" << std::endl;
      return true;
    },

    [&]() -> bool
    {
      // condition of state finish
      // std::cout << "99. emergency_stop_state finish condition check" << std::endl;

      return true;
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
