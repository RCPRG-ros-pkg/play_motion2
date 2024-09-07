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

  get_motion_info_client_ = this->create_client<GetMotionInfo>("/play_motion2/get_motion_info");
  is_motion_ready_client_ = this->create_client<IsMotionReady>("/play_motion2/is_motion_ready");
  list_motions_client_ = this->create_client<ListMotions>("/play_motion2/list_motions");

  add_motion_client_ = this->create_client<AddMotion>("/play_motion2/add_motion");
  remove_motion_client_ = this->create_client<RemoveMotion>("/play_motion2/remove_motion");
}

PlayMotion2Client::~PlayMotion2Client()
{
  play_motion2_client_.reset();

  get_motion_info_client_.reset();
  is_motion_ready_client_.reset();
  list_motions_client_.reset();
  add_motion_client_.reset();
  remove_motion_client_.reset();
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

std::vector<std::string> PlayMotion2Client::list_motions()
{
  std::vector<std::string> motions;

  if (!list_motions_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for ListMotions service");
    return motions;
  }

  auto request = std::make_shared<ListMotions::Request>();
  auto future = list_motions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to list motions");
    return motions;
  }

  return future.get()->motion_keys;
}

bool PlayMotion2Client::is_motion_ready(const std::string & motion_key)
{
  if (!is_motion_ready_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for IsMotionReady service");
    return false;
  }

  auto request = std::make_shared<IsMotionReady::Request>();
  request->motion_key = motion_key;
  auto future = is_motion_ready_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to check if motion is ready");
    return false;
  }

  return future.get()->is_ready;
}

MotionInfo PlayMotion2Client::get_motion_info(const std::string & motion_key)
{
  if (!get_motion_info_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for GetMotionInfo service");
    return MotionInfo();
  }

  auto request = std::make_shared<GetMotionInfo::Request>();
  request->motion_key = motion_key;
  auto future = get_motion_info_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get motion info");
    return MotionInfo();
  }

  auto motion = future.get()->motion;

  MotionInfo motion_info;
  motion_info.key = motion.key;
  motion_info.joints = motion.joints;
  motion_info.positions = motion.positions;
  motion_info.times = motion.times_from_start;
  motion_info.name = motion.name;
  motion_info.description = motion.description;
  motion_info.usage = motion.usage;

  return motion_info;
}

bool PlayMotion2Client::add_motion(const MotionMsg & motion_msg)
{
  if (!add_motion_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for AddMotion service");
    return false;
  }

  auto request = std::make_shared<AddMotion::Request>();
  request->motion = motion_msg;
  auto future = add_motion_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to add motion");
    return false;
  }

  return future.get()->success;
}

bool PlayMotion2Client::remove_motion(const std::string & motion_key)
{
  if (!remove_motion_client_->wait_for_service(kTimeout)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for RemoveMotion service");
    return false;
  }

  auto request = std::make_shared<RemoveMotion::Request>();
  request->motion_key = motion_key;
  auto future = remove_motion_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), future, kTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to remove motion");
    return false;
  }

  return future.get()->success;
}
}  // namespace play_motion2
