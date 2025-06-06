// Copyright 2024 clober
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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char * argv[])
{
  std::string net_if_str = argv[1];

  if (net_if_str != "--net_if") {
    std::cout << "Usage: " << argv[0] << " --net_if {Network Interface}" << std::endl;
    exit(-1);
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<ArmControllerNode>(argv[2]);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
