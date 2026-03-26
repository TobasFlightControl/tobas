#pragma once

#include <mutex>

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

namespace tobas
{
namespace gazebo
{
/* Gazeboメッセージを1通だけ取得する． */
template <typename MsgT, typename RepT = int64_t, typename RatioT = std::milli>
bool waitForMessage(
  MsgT& _msg_out,
  const std::string& _topic,
  std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1))
{
  gz::transport::Node node;
  std::mutex mutex;
  std::condition_variable cv;
  bool got = false;

  // コールバック (最初の1通だけ採用)
  const std::function<void(const MsgT&)> cb = [&](const MsgT& msg)
  {
    const std::lock_guard lock(mutex);
    if (got) {
      return;  // 2通目以降は無視
    }
    _msg_out = msg;
    got = true;
    cv.notify_one();
  };

  // 購読開始
  if (!node.Subscribe(_topic, cb)) {
    gzerr << "Failed to subscribe \"" << _topic << "\"." << std::endl;
    return false;
  }

  // メッセージの受信待ち
  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto wait_cb = [&] { return got; };
    if (_timeout.count() > 0) {
      cv.wait_for(lock, _timeout, wait_cb);
    }
    else {
      cv.wait(lock, wait_cb);
    }
  }

  // 購読解除
  node.Unsubscribe(_topic);

  return got;
}
}  // namespace gazebo
}  // namespace tobas
