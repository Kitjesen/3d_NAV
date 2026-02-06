#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>


namespace remote_monitoring {
namespace core {

/**
 * @brief IdempotencyCache stores RPC responses to ensure duplicate requests
 *        (due to network retries) return the same result without re-executing
 * logic.
 *
 * Thread-safe.
 */
class IdempotencyCache {
public:
  struct CacheEntry {
    std::string response_data; // Serialized protobuf response
    std::chrono::steady_clock::time_point timestamp;
    bool is_error; // If we want to cache errors differently? usually we cache
                   // the Service response as-is.
  };

  /**
   * @param ttl_seconds Duration to keep cached responses. Default 10 minutes.
   */
  explicit IdempotencyCache(int ttl_seconds = 600)
      : ttl_(std::chrono::seconds(ttl_seconds)) {}

  /**
   * @brief Try to get a cached response for a request ID.
   * @param request_id Unique ID from client request.
   * @param[out] out_response Serialized response data if found.
   * @return true if found and valid, false otherwise.
   */
  bool TryGet(const std::string &request_id, std::string *out_response) {
    if (request_id.empty())
      return false;

    std::lock_guard<std::mutex> lock(mutex_);

    // Lazy cleanup on access (probabilistic or minimal overhead)
    // For production, maybe a separate thread is better, but this remains
    // simple.

    auto it = cache_.find(request_id);
    if (it != cache_.end()) {
      if (IsExpired(it->second)) {
        cache_.erase(it);
        return false;
      }
      if (out_response) {
        *out_response = it->second.response_data;
      }
      return true;
    }
    return false;
  }

  /**
   * @brief Store a response for a request ID.
   */
  void Set(const std::string &request_id, const std::string &response_data) {
    if (request_id.empty())
      return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Prune expired periodically or on insert?
    // Let's do a quick prune of random/old items if size gets too big,
    // but for now, simple lazy expiration is fine unless high QPS.
    // Given robot control is low QPS, this is safe.

    CacheEntry entry;
    entry.response_data = response_data;
    entry.timestamp = std::chrono::steady_clock::now();
    entry.is_error = false;

    cache_[request_id] = entry;
  }

  // Optional: Explicit cleanup method
  void Cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = cache_.begin(); it != cache_.end();) {
      if (IsExpired(it->second)) {
        it = cache_.erase(it);
      } else {
        ++it;
      }
    }
  }

private:
  bool IsExpired(const CacheEntry &entry) const {
    return (std::chrono::steady_clock::now() - entry.timestamp) > ttl_;
  }

  std::mutex mutex_;
  std::unordered_map<std::string, CacheEntry> cache_;
  std::chrono::seconds ttl_;
};

} // namespace core
} // namespace remote_monitoring
