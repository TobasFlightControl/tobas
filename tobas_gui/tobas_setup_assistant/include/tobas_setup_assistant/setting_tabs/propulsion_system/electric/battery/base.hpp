#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class BatteryWidget_Base : public QWidget
{
  Q_OBJECT

  using self = BatteryWidget_Base;
  using super = QWidget;

public:
  virtual const char* name() const = 0;

  virtual bool isValid() = 0;

  virtual YAML::Node dump() = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* 公称電圧 [V] */
  virtual double nominalVoltage() = 0;

  /* 最大電圧 [V] */
  virtual double maxVoltage() = 0;

  /* 放電特性が急激に変化する電圧 [V] */
  virtual double sagVoltage() = 0;

  /* 最大連続電流 [A] */
  virtual double maxCurrent() = 0;

  /* 電気容量 [As] */
  virtual double capacity() = 0;

  /* 内部抵抗値 [Ω] */
  virtual double internalRegistance() = 0;
};
};  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
