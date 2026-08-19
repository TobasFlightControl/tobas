#pragma once

#include <QWidget>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class WingsWidget : public QWidget
{
  Q_OBJECT

  using self = WingsWidget;
  using super = QWidget;

public:
  explicit WingsWidget();
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

private:
  int index_ = 0;
  qt::TabWidget* tabs_;

  void addWingWidget();
  void removeWingWidget();
  void renameWingWidget(const int index);
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
