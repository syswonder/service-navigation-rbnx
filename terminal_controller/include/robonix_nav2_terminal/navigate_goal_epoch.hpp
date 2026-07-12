#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace robonix_nav2_terminal
{

class NavigateGoalEpoch
{
public:
  void observe(const action_msgs::msg::GoalStatusArray & statuses)
  {
    const action_msgs::msg::GoalStatus * active = nullptr;
    for (const auto & status : statuses.status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
      {
        active = &status;
      }
    }
    if (active == nullptr) {
      return;
    }
    std::string uuid;
    uuid.reserve(active->goal_info.goal_id.uuid.size() * 2);
    static constexpr char hex[] = "0123456789abcdef";
    for (const auto value : active->goal_info.goal_id.uuid) {
      uuid.push_back(hex[value >> 4]);
      uuid.push_back(hex[value & 0x0f]);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (uuid != active_uuid_) {
      active_uuid_ = std::move(uuid);
      epoch_.fetch_add(1, std::memory_order_release);
    }
  }

  uint64_t value() const
  {
    return epoch_.load(std::memory_order_acquire);
  }

private:
  mutable std::mutex mutex_;
  std::string active_uuid_;
  std::atomic<uint64_t> epoch_{0};
};

}  // namespace robonix_nav2_terminal
