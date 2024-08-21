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

#include "play_motion2/client.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"

#include "rclcpp_action/create_client.hpp"
#include "rclcpp/executors.hpp"


namespace play_motion2
{
using namespace std::chrono_literals;
const auto kTimeout = 10s;

PlayMotion2Client::PlayMotion2Client(const std::string & name)
: Node(name)
{
  play_motion2_client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");
}

PlayMotion2Client::~PlayMotion2Client()
{
  play_motion2_client_.reset();
}

bool PlayMotion2Client::run_motion(
  const std::string & motion_name,
  const bool skip_planning,
  const std::chrono::seconds & motion_timeout)
{
  if (!play_motion2_client_->wait_for_action_server(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for PlayMotion2 server");
    return false;
  }

  auto goal = PlayMotion2::Goal();
  goal.motion_name = motion_name;
  goal.skip_planning = skip_planning;
  auto goal_future = play_motion2_client_->async_send_goal(goal);

  if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot send goal to PlayMotion2 server");
    return false;
  }

  const auto goal_handle = goal_future.get();

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by PlayMotion2 server");
    return false;
  }

  auto execution_future = play_motion2_client_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(
      shared_from_this(), execution_future, motion_timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute PlayMotion2 goal");
    play_motion2_client_->async_cancel_goal(goal_handle);
    return false;
  }

  switch (execution_future.get().code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Motion execution completed");
      return true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Motion execution failed");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Motion execution cancelled");
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      throw std::runtime_error("Got unknown result code");
  }
  return false;
}
}  // namespace play_motion2
