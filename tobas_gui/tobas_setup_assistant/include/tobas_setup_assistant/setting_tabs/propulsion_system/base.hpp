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

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;

public:
  explicit BasePropulsionSystemWidget();

  virtual const char* name() const = 0;

  virtual void reset() = 0;
  virtual void updateInternalDataStructures() = 0;
  virtual bool isValid() = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual tobas::propulsion_system_t type() const = 0;
  virtual int numUnits() const = 0;

  virtual QString linkName(int index) const = 0;
  virtual bool isTiltRotor(int index) const = 0;
  virtual QString tiltJointName(int index) const = 0;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
