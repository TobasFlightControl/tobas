#pragma once

#include <QWidget>

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./wing.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
class WingsWidget : public QWidget
{
  Q_OBJECT

  using self = WingsWidget;
  using super = QWidget;

  static constexpr char kMainWing[] = "main_wing";

Q_SIGNALS:
  void tabAdded(QString tab_name);
  void tabRemoved(int index);
  void tabRenamed(int index, QString new_name);

public:
  explicit WingsWidget();

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid() const;

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double cruiseSpeed() const;
  QString requiredWingName() const;
  WingWidget* getWing(const int& index) const;
  WingWidget* getMainWing() const;
  int count() const;

private:
  int index_ = 0;
  ParamGetterWidget_DoubleSpinBox* cruise_speed_;
  qt::TabWidget* tabs_;

  void addWingWidget();
  void addWingWidgetWithName(const std::string& wing_name);
  void removeWingWidget();
  void renameWingWidget(const int index);
};
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
