#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_drone_core/propulsion_system/type.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
class BasePropulsionSystemWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;

public:
  virtual const char* name() const = 0;

  virtual void updateInternalDataStructures() = 0;
  virtual bool isValid() = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual tobas::PropulsionSystem type() const = 0;
  virtual int numUnits() const = 0;

  virtual QString linkName(int index) const = 0;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
