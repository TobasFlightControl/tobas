#pragma once

#include <ros/ros.h>

namespace tobas
{
/**
 * @brief あらゆるノードの規定ノード．プログラミングの自由度を下げて見通しを良くすることが目的．
 */
class BaseNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  const std::string name_;

  explicit BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& name);

  virtual void getRosParams() = 0;
  virtual void registerPublishers() = 0;
  virtual void registerSubscribers() = 0;

  inline std::string ns();

  /**
   * @brief コマンドレベルを更新する．
   *
   * @param cur_level 現在のコマンドレベル．
   * @param new_level 指令されたコマンドレベル．
   *
   * @return 指令されたコマンドが有効な場合にtrueを返す．
   */
  bool updateCommandLevel(uint8_t& cur_level, const uint8_t& new_level);

  /* Alias for ros::TransportHints().reliable().tcpNoDelay(). */
  static ros::TransportHints tcpNoDelay(const bool& nodelay = true);
};

inline std::string BaseNode::ns()
{
  return nh_.getNamespace() + "/";
}
}  // namespace tobas
