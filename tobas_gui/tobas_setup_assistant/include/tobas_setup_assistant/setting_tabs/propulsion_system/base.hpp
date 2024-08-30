#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace gui
{
namespace setup_assistant
{
template <typename Derived>
class BaseSelectedLinkSettingWidget : public QWidget
{
public:
  virtual const char* name() = 0;
  virtual bool isValid() = 0;
  virtual void copyFrom(const Derived* src) = 0;

  virtual YAML::Node dump() = 0;
  virtual void load(const YAML::Node& node) = 0;
};
}  // namespace setup_assistant
}  // namespace gui
