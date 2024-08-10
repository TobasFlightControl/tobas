#pragma once

#include <map>
#include <chrono>
#include <mutex>

// Log throttle data structure
static std::map<std::string, std::chrono::steady_clock::time_point> g_log_throttle;
// Mutex to protect logThrottle. Without it, the formatting breaks down.
static std::mutex g_log_throttle_mutex;

#define GZ_LOG_THROTTLE(period, msg, os)                                                                               \
  {                                                                                                                    \
    std::string id = std::string(__FILE__) + ":" + std::to_string(__LINE__);                                           \
    const auto now = std::chrono::steady_clock::now();                                                                 \
    std::lock_guard<std::mutex> lock(g_log_throttle_mutex);                                                            \
    auto it = g_log_throttle.find(id);                                                                                 \
    if (it == g_log_throttle.end())                                                                                    \
    {                                                                                                                  \
      g_log_throttle[id] = now;                                                                                        \
      os << msg << std::endl;                                                                                          \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      const auto diff = std::chrono::duration_cast<double>(now - it->second).count();                                  \
      if (diff > period)                                                                                               \
      {                                                                                                                \
        it->second = now;                                                                                              \
        os << msg << std::endl;                                                                                        \
      }                                                                                                                \
    }                                                                                                                  \
  }

#define GZ_MSG_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzmsg)
#define GZ_WARN_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzwarn)
#define GZ_ERROR_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzerr)
