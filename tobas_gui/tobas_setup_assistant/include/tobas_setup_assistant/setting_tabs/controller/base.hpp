#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_constants/constants.hpp>

namespace gui
{
namespace setup_assistant
{
class BaseControllerWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual const char* description() const = 0;
  virtual QString controllerPackage() const = 0;
  virtual QString pluginName() const = 0;

  virtual tobas::rc_command_t stabilizeModeCommand() const = 0;
  virtual tobas::rc_command_t acrobatModeCommand() const = 0;

  /* 静的プライベートROSパラメータ． */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /**
   * @brief ハードウェアの構造のみから，制御器が適用可能かどうかを返す．
   * @note 実験データによるモータの設定など，個別の設定方法に依存してはならない．
   */
  virtual bool isApplicable() = 0;

  /* ユーザ設定が有効な場合にtrueを返す． */
  virtual bool isValid() = 0;
};
}  // namespace setup_assistant
}  // namespace gui
