#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/multirotor_real/barometer_handler.hpp"

#define TIMER_PERIOD 0.01
#define WAIT_TIME 0.008

// MS5611(http://www.kyohritsu.jp/eclib/OTHER/DATASHEET/SENSOR/ms561101ba03.pdf)
// 正確度と精度(https://www.hitachi-hightech.com/jp/ja/knowledge/semiconductor/room/manufacturing/accuracy-precision.html)
// 精度(precision)がノイズにあたり，それ関する情報は無かった
#define BAR_NOISE_STD 1.  // TODO: センサの精度を計測する

using namespace std;

BarometerHandler::BarometerHandler()
{
  barometer_.initialize();

  bar_msg_.variance = dh_std::sqr(BAR_NOISE_STD);

  string drone_name = dh_ros::getParam<string>("/drone_name");
  bar_pub_ = nh_.advertise<BarMsg>("/" + drone_name + "/air_pressure", 1);

  timer_ = nh_.createTimer(ros::Duration(TIMER_PERIOD), &BarometerHandler::timerCb, this);
}

void BarometerHandler::timerCb(const ros::TimerEvent&)
{
  barometer_.refreshPressure();
  ros::Duration(WAIT_TIME).sleep();  // Waiting for pressure data ready
  barometer_.readPressure();
  barometer_.calculatePressureAndTemperature();

  bar_msg_.fluid_pressure = barometer_.getPressure() * 100;  // mbar -> Pa
  bar_pub_.publish(bar_msg_);
}
