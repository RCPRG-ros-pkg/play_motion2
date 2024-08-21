// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "play_motion2_msgs/action/play_motion2.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp/node.hpp"

#ifndef PLAY_MOTION2__CLIENT_HPP_
#define PLAY_MOTION2__CLIENT_HPP_

namespace play_motion2
{
class PlayMotion2Client : public rclcpp::Node
{
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;

public:
  explicit PlayMotion2Client(const std::string & name = "play_motion2_client");
  ~PlayMotion2Client();

  bool run_motion(
    const std::string & motion_name,
    const bool skip_planning,
    const std::chrono::seconds & motion_timeout = std::chrono::seconds(120));

private:
  rclcpp_action::Client<PlayMotion2>::SharedPtr play_motion2_client_;
};
}  // namespace play_motion2

#endif  // PLAY_MOTION2__CLIENT_HPP_
