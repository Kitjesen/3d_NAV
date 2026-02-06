#include "remote_monitoring/core/event_buffer.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace remote_monitoring {
namespace core {

EventBuffer::EventBuffer(size_t max_size) : max_size_(max_size) {}

void EventBuffer::AddEvent(robot::v1::EventType type,
                           robot::v1::EventSeverity severity,
                           const std::string &title,
                           const std::string &description) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  robot::v1::Event event;
  event.set_event_id(GenerateEventId());
  event.set_type(type);
  event.set_severity(severity);
  event.set_title(title);
  event.set_description(description);
  
  const auto now = std::chrono::system_clock::now();
  const auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  event.mutable_timestamp()->set_seconds(now_sec);
  
  buffer_.push_back(event);
  
  // 保持缓冲区大小
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }

  cv_.notify_all();
}

std::vector<robot::v1::Event> EventBuffer::GetEventsSince(const std::string &last_event_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<robot::v1::Event> result;
  bool found_last = last_event_id.empty();
  
  for (const auto &event : buffer_) {
    if (found_last) {
      result.push_back(event);
    } else if (event.event_id() == last_event_id) {
      found_last = true;
    }
  }
  
  return result;
}

std::vector<robot::v1::Event> EventBuffer::GetLatestEvents(size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<robot::v1::Event> result;
  const size_t start = buffer_.size() > count ? buffer_.size() - count : 0;
  
  for (size_t i = start; i < buffer_.size(); ++i) {
    result.push_back(buffer_[i]);
  }
  
  return result;
}

void EventBuffer::AckEvent(const std::string &event_id) {
  (void)event_id;
  // 可选：记录已确认的事件（用于审计）
}

bool EventBuffer::WaitForEventAfter(const std::string &last_event_id,
                                    robot::v1::Event *event,
                                    std::chrono::milliseconds timeout) {
  if (event == nullptr) {
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, timeout, [&]() {
        return NextIndexLocked(last_event_id) != buffer_.size();
      })) {
    return false;
  }

  const size_t idx = NextIndexLocked(last_event_id);
  if (idx >= buffer_.size()) {
    return false;
  }

  *event = buffer_[idx];
  return true;
}

std::string EventBuffer::GenerateEventId() {
  std::ostringstream oss;
  const auto now = std::chrono::system_clock::now();
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
  oss << std::hex << std::setfill('0') << std::setw(12) << now_ms
      << std::setw(6) << next_sequence_++;
  return oss.str();
}

size_t EventBuffer::NextIndexLocked(const std::string &last_event_id) const {
  if (buffer_.empty()) {
    return buffer_.size();
  }

  if (last_event_id.empty()) {
    return 0;
  }

  for (size_t i = 0; i < buffer_.size(); ++i) {
    if (buffer_[i].event_id() == last_event_id) {
      const size_t next_idx = i + 1;
      return next_idx < buffer_.size() ? next_idx : buffer_.size();
    }
  }

  // last_event_id 已经不在 ring buffer 中时，从最早可用事件继续。
  return 0;
}

}  // namespace core
}  // namespace remote_monitoring
