#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_real/barometer_handler.hpp"

#define TIMER_PERIOD 0.01
#define WAIT_TIME 0.008

// MS5611(http://www.kyohritsu.jp/eclib/OTHER/DATASHEET/SENSOR/ms561101ba03.pdf)
// 正確度と精度(https://www.hitachi-hightech.com/jp/ja/knowledge/semiconductor/room/manufacturing/accuracy-precision.html)
// 精度(precision)がノイズにあたり，それ関する情報は無かった
#define BAR_NOISE_STD 1.  // TODO: センサの精度を計測する

using namespace std;

BarometerHandler::BarometerHandler() : super()
{
  getRosParams();

  barometer_.initialize();
  bar_msg_.variance = dh_std::sqr(BAR_NOISE_STD);

  registerPublishers();
  registerSubscribers();
  createTimers();
}

void BarometerHandler::getRosParams()
{
}

void BarometerHandler::registerPublishers()
{
  bar_pub_ = nh_.advertise<BarMsg>("air_pressure", 1);
}

void BarometerHandler::registerSubscribers()
{
}

void BarometerHandler::createTimers()
{
  main_loop_timer_ =
    nh_.createTimer(ros::Duration(TIMER_PERIOD), &BarometerHandler::mainLoopTimerCb, this);
}

void BarometerHandler::checkTopicsTimerCb(const ros::TimerEvent& event)
{
}

void BarometerHandler::mainLoopTimerCb(const ros::TimerEvent&)
{
  barometer_.refreshPressure();
  ros::Duration(WAIT_TIME).sleep();  // Waiting for pressure data ready
  barometer_.readPressure();
  barometer_.calculatePressureAndTemperature();

  bar_msg_.fluid_pressure = barometer_.getPressure() * 100;  // mbar -> Pa
  bar_pub_.publish(bar_msg_);
}
