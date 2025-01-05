#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
class BaseSelectedLinkSettingWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual bool isValid() = 0;
  virtual void copyFrom(const BaseSelectedLinkSettingWidget* src) = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
