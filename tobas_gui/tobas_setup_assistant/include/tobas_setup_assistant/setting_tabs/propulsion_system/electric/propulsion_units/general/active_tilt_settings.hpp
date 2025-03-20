#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>
#include <QComboBox>

#include "tobas_setup_assistant/robot_info.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class ActiveTiltSettingsWidget : public QWidget
{
  Q_OBJECT

  using self = ActiveTiltSettingsWidget;
  using super = QWidget;

  static constexpr char kIsTiltKey[] = "is_tilt";
  static constexpr char kTiltJointNameKey[] = "tilt_joint_name";

public:
  explicit ActiveTiltSettingsWidget(const RobotInfo& robot, Signals& _signals, const QString& link_name);

  bool isValid();
  void copyFrom(const ActiveTiltSettingsWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  bool isTiltRotor() const;
  QString tiltJointName() const;

private:
  const RobotInfo& robot_;
  Signals& signals_;
  const QString link_name_;

  QCheckBox* is_tilt_;
  QComboBox* tilt_joint_name_;

private Q_SLOTS:
  void onIsTiltCheckBoxToggled(bool checked);
  void onTiltJointNameChanged(const QString& joint_name);
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
