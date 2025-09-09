#pragma once

#include <QButtonGroup>
#include <QRadioButton>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace rc
{
class HostWidget : public QWidget
{
  Q_OBJECT

  using self = HostWidget;
  using super = QWidget;

  static constexpr char kIsCheckedKey[] = "is_checked";
  static constexpr char kSettingsKey[] = "settings";

public:
  explicit HostWidget();

  bool isValid();

  YAML::Node dump();
  void load(const YAML::Node& node);

  QString host() const;

private:
  QVector<QRadioButton*> buttons_;
  QVector<BaseHostWidget*> widgets_;

  void addRow(qt::FormLayout* form, QButtonGroup* btn_group, BaseHostWidget* widget);

  void updateEnabled();

  int rowCount() const;
  int findCurrentRow() const;

private Q_SLOTS:
  void onButtonGroupIdClicked();
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
