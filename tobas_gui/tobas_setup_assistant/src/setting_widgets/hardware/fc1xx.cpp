#include "tobas_setup_assistant/setting_tabs/hardware/fc1xx.hpp"

#include <QVBoxLayout>

#include <tobas_std_tools/universal_constants.hpp>

namespace gui
{
namespace sa
{
namespace hw
{
T1Widget::T1Widget()
{
}

const char* T1Widget::name() const
{
  return "Tobas FC1xx";
}

YAML::Node T1Widget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void T1Widget::load(const YAML::Node&)
{
}

bool T1Widget::isValid()
{
  return true;
}

const char* T1Widget::hardwarePackage() const
{
  return "tobas_fc1xx_ros";
}

int T1Widget::imuUpdateRate() const
{
  return 400;
}

double T1Widget::gyroNoiseDensity() const
{
  return 0.011 * tbs::kDeg2Rad;  // ISM330DLC
}

double T1Widget::gyroRandomWalk() const
{
  return 0.;  // TODO
}

int T1Widget::gyroBiasCorrTime() const
{
  return 1000;  // TODO
}

double T1Widget::accNoiseDensity() const
{
  return 1.7e-4 * tbs::kGravity;  // ISM330DLC
}

double T1Widget::accRandomWalk() const
{
  return 0.;  // TODO
}

int T1Widget::accBiasCorrTime() const
{
  return 300;  // TODO
}

int T1Widget::magUpdateRate() const
{
  return 50;
}

double T1Widget::magNoiseStddev() const
{
  return 4.6e-3;  // IIS2MDC
}

double T1Widget::magHardBiasNorm() const
{
  return 0.03;  // IIS2MDCの最大値は6000nTだが，キャリブレーションを前提としてそれより低めに設定．
}

int T1Widget::presUpdateRate() const
{
  return 50;
}

double T1Widget::presNoiseStddev() const
{
  return 1.16;  // ILPS22QS (Table 23: FS = 1260, AVG = 32, ODR/4)
}

int T1Widget::gnssUpdateRate() const
{
  return 20;
}

double T1Widget::gnssHorizontalPositionAccuracy() const
{
  return 1.5;  // ZED-F9P
}

double T1Widget::gnssVerticalPositionAccuracy() const
{
  return 2.0;  // ZED-F9P
}

double T1Widget::gnssHorizontalVelocityStddev() const
{
  return 0.05;  // FIXME: 精度 (Accurasy) と標準偏差は異なる
}

double T1Widget::gnssVerticalVelocityStddev() const
{
  return 0.05;  // FIXME: 精度 (Accurasy) と標準偏差は異なる
}

int T1Widget::numPwmChannels() const
{
  return 8;
}

int T1Widget::numDShotChannels() const
{
  return 8;
}
}  // namespace hw
}  // namespace sa
}  // namespace gui
