#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_drone_core/propulsion_system/type.hpp>
#include <tobas_qt_tools/widgets/tab_widget.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
class BasePropulsionSystemWidget : public qt::TabWidget
{
  Q_OBJECT

Q_SIGNALS:
  void linkAdded(const QString& link_name);
  void linkRemoved(const QString& link_name);
  void isTiltStateChanged(const QString& link_name, bool is_tilt);
  void tiltJointNameChanged(const QString& link_name, const QString& tilt_joint_name);

public:
  virtual const char* name() const = 0;

  virtual void updateInternalDataStructures() = 0;
  virtual bool isValid() = 0;

  virtual YAML::Node dump() = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual tobas::propulsion_system_t type() const = 0;
  virtual int numUnits() const = 0;

  virtual QString linkName(int index) const = 0;
  virtual bool isTiltRotor(int index) const = 0;
  virtual QString tiltJointName(int index) const = 0;

protected:
  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
