#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_constants/rc_command.hpp>

#include "tobas_setup_assistant/frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class BaseControllerWidget : public QWidget
{
  Q_OBJECT

public:
  virtual FrameType frameType() const = 0;
  virtual QString controllerPackage() const = 0;
  virtual QString pluginName() const = 0;

  virtual RcCommand acrobatModeCommand() const = 0;
  virtual RcCommand stabilizeModeCommand() const = 0;
  virtual RcCommand loiterModeCommand() const = 0;

  /* 静的プライベートROSパラメータ． */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* ユーザ設定が有効な場合にtrueを返す． */
  virtual bool isValid() = 0;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
