#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "common.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer {
public:
  explicit EventBuffer(size_t max_size = 1000);
  
  // 添加事件（自动生成 event_id）
  void AddEvent(robot::v1::EventType type,
                robot::v1::EventSeverity severity,
                const std::string &title,
                const std::string &description);
  
  // 获取从 last_event_id 之后的所有事件（回放）
  std::vector<robot::v1::Event> GetEventsSince(const std::string &last_event_id);
  
  // 获取最新 N 个事件
  std::vector<robot::v1::Event> GetLatestEvents(size_t count);
  
  // 标记事件已被客户端确认
  void AckEvent(const std::string &event_id);

  // 阻塞等待一条新事件（event_id > last_event_id）。
  bool WaitForEventAfter(const std::string &last_event_id, robot::v1::Event *event,
                         std::chrono::milliseconds timeout);

private:
  std::string GenerateEventId();
  size_t NextIndexLocked(const std::string &last_event_id) const;
  
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<robot::v1::Event> buffer_;
  size_t max_size_;
  uint64_t next_sequence_{1};
};

}  // namespace core
}  // namespace remote_monitoring
