#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>
#include <QButtonGroup>

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
class SpeedLimitWidget_Base : public QWidget
{
  Q_OBJECT

  static constexpr char kIsCheckedKey[] = "is_checked";
  static constexpr char kValueKey[] = "value";

public:
  void initialize(QButtonGroup* ckb_group);

  virtual const char* name() const = 0;

  virtual void onInit() = 0;

  virtual bool isValid() = 0;

  /* Maximum rotation speed [rad/s] */
  virtual double maxRotSpeed() const = 0;

  void copyFrom(const SpeedLimitWidget_Base* src);

  YAML::Node dump();
  void load(const YAML::Node& node);

  bool isChecked() const;
  void setChecked(bool checked);

protected:
  qt::DoubleSpinBox* spinbox_;

private:
  QCheckBox* checkbox_;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
