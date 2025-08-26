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

  static constexpr int kButtonCol = 0;
  static constexpr int kLabelCol = 1;
  static constexpr int kWidgetCol = 2;

  struct Line
  {
    QRadioButton* button;
    BaseHostWidget* widget;
  };

public:
  explicit HostWidget();

  bool isValid();

  YAML::Node dump();
  void load(const YAML::Node& node);

  QString host() const;

private:
  QVector<Line> lines_;

  void addRow(qt::FormLayout* form, QButtonGroup* group, BaseHostWidget* widget);

  void updateEnabled();

  int rowCount() const;
  int findCurrentRow() const;

  QRadioButton* getRadio(int row);
  const QRadioButton* getRadio(int row) const;

  BaseHostWidget* getWidget(int row);
  const BaseHostWidget* getWidget(int row) const;

private Q_SLOTS:
  void onButtonGroupIdClicked();
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
