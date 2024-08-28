#pragma once

#include <yaml-cpp/yaml.h>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "../param_getters/base.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class BaseSettingWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = BaseSettingWidget;
  using super = qt::ScrollArea;

protected:
  static constexpr int kDescriptionHeight = 100;

public:
  explicit BaseSettingWidget(SetupAssistant* main);

  virtual void initialize();

  virtual const char* name() = 0;
  virtual const char* title() = 0;
  virtual const char* description() = 0;

  /* 初期化時の処理． */
  virtual void onInit() = 0;

  /* タブが開かれた時に呼ばれるコールバック．表示内容が他のタブの状態に依存する場合に使う． */
  virtual void onOpened() = 0;

  /* URDFの変化に合わせて内部状態を更新する． */
  virtual void updateInternalDataStructures() = 0;

  /* ユーザ設定に問題がない場合にTrueを返す． */
  virtual bool isValid() = 0;

  /* ユーザ設定を書き出す． */
  virtual YAML::Node dump() = 0;

  /* ユーザ設定を読み込む． */
  virtual void load(const YAML::Node& node) = 0;

protected:
  SetupAssistant* main_;

  void addTitleAndDescription();
  void addWidget(QWidget* widget);
  void addLayout(QLayout* layout);

private:
  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;
};
};  // namespace setup_assistant
}  // namespace gui
