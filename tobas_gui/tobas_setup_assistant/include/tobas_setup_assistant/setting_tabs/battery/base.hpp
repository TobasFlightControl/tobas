#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/widget.hpp>

namespace gui
{
namespace setup_assistant
{
class BatteryWidget_Base : public qt::Widget
{
  Q_OBJECT

  using self = BatteryWidget_Base;
  using super = qt::Widget;

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
};  // namespace setup_assistant
}  // namespace gui
