#pragma once

#include <yaml-cpp/yaml.h>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "../param_getters/base.hpp"

namespace gui
{
namespace sa
{
class BaseSettingWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = BaseSettingWidget;
  using super = qt::ScrollArea;

protected:
  static constexpr int kDescriptionHeight = 100;

public:
  explicit BaseSettingWidget();

  virtual const char* name() const = 0;
  virtual const char* title() const = 0;
  virtual const char* description() const = 0;

  /* タブが開かれた時に呼ばれるコールバック．表示内容が他のタブの状態に依存する場合に使う． */
  virtual void onOpened() = 0;

  /* URDFの変化に合わせて内部状態を更新する． */
  virtual void updateInternalDataStructures() = 0;

  /* ユーザ設定に問題がない場合にTrueを返す． */
  virtual bool isValid() = 0;

  /* ユーザ設定を書き出す． */
  virtual YAML::Node dump() const = 0;

  /* ユーザ設定を読み込む． */
  virtual void load(const YAML::Node& node) = 0;

protected:
  void addWidget(QWidget* widget);
  void addWidgetCenter(QWidget* widget);
  void addLayout(QLayout* layout);
  void addStretch();
  void addSpacing(int size);

private:
  QLabel* title_;
  qt::DescriptionWidget* description_;

  QVBoxLayout* header_rows_;
  QVBoxLayout* content_rows_;

private Q_SLOTS:
  void initialize();
};
};  // namespace sa
}  // namespace gui
