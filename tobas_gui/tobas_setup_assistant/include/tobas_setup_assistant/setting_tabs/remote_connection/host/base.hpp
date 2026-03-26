#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
class BaseHostWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* label() const = 0;

  virtual bool isValid() = 0;

  virtual YAML::Node dump() = 0;
  virtual void load(const YAML::Node& node) = 0;

  virtual QString host() const = 0;
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
