#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>
#include <QButtonGroup>

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
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

protected:
  qt::DoubleSpinBox* spinbox_;

private Q_SLOTS:
  void onCheckBoxToggled(bool toggled);

private:
  QCheckBox* checkbox_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
