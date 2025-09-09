#pragma once

#include <QButtonGroup>
#include <QRadioButton>
#include <rclcpp/node.hpp>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "./base.hpp"

namespace gui
{
namespace sim
{
class WorldWidget : public QWidget
{
  Q_OBJECT

  using self = WorldWidget;
  using super = QWidget;

public:
  explicit WorldWidget(rclcpp::Node::SharedPtr node);

  std::filesystem::path worldPath() const;

private:
  QVector<QRadioButton*> buttons_;
  QVector<BaseWorldWidget*> widgets_;

  void addRow(qt::FormLayout* form, QButtonGroup* btn_group, BaseWorldWidget* widget, const QString& label);

  void updateEnabled();

  int rowCount() const;
  int findCurrentRow() const;

private Q_SLOTS:
  void onButtonGroupIdClicked();
};
}  // namespace sim
}  // namespace gui
