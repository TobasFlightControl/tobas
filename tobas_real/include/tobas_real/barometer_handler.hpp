#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <Common/MS5611.h>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BarometerHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;    // [Hz]

  // MS5611(http://www.kyohritsu.jp/eclib/OTHER/DATASHEET/SENSOR/ms561101ba03.pdf)
  // 正確度と精度(https://www.hitachi-hightech.com/jp/ja/knowledge/semiconductor/room/manufacturing/accuracy-precision.html)
  // 精度(precision)がノイズにあたり，それ関する情報は無かった
  // TODO: 実際のデータには白色ノイズモデルでは表せないバイアスが乗っているため，モデルから考え直す
  static constexpr double kBarNoiseStd = 10.;

  using super = tobas::BaseNode;

  using BarMsg = sensor_msgs::FluidPressure;

public:
  explicit BarometerHandler();

  void run();

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

  // PubSub
  ros::Publisher bar_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
