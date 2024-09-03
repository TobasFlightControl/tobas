#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace gui
{
namespace setup_assistant
{
class BaseHardwareWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual const char* description() const = 0;
  virtual const char* hardwarePackage() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual bool isValid() = 0;
};
}  // namespace setup_assistant
}  // namespace gui
