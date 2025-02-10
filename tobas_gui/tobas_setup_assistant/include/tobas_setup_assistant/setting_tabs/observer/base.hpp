#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace gui
{
namespace sa
{
class BaseObserverWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual const char* description() const = 0;
  virtual QString observerPackage() const = 0;
  virtual QString pluginName() const = 0;

  /* 静的プライベートROSパラメータ． */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* ユーザ設定が有効な場合にtrueを返す． */
  virtual bool isValid() = 0;
};
}  // namespace sa
}  // namespace gui
