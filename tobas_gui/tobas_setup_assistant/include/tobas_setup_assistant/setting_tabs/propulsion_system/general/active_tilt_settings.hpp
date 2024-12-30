#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>
#include <QComboBox>

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class ActiveTiltSettingsWidget : public QWidget
{
  Q_OBJECT

  static constexpr char kIsTiltKey[] = "is_tilt";
  static constexpr char kTiltJointNameKey[] = "tilt_joint_name";

public:
  explicit ActiveTiltSettingsWidget();

  bool isValid();
  void copyFrom(const ActiveTiltSettingsWidget* src);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  bool isTiltRotor() const;
  QString tiltJointName() const;

private:
  QCheckBox* is_tilt_;
  QComboBox* tilt_joint_name_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
