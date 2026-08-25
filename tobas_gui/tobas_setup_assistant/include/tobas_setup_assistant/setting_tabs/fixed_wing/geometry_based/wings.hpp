#pragma once

#include <QWidget>

#include <tobas_qt_tools/widgets/tab_widget.hpp>
#include <tobas_setup_assistant/param_getters/double_spin_box.hpp>
#include <tobas_setup_assistant/param_getters/double_range.hpp>
#include <tobas_setup_assistant/setting_tabs/fixed_wing/geometry_based/wing.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace gb
{
class WingsWidget : public QWidget
{
  Q_OBJECT

  using self = WingsWidget;
  using super = QWidget;

  static constexpr char kCruiseSpeedLabel[] = "Cruise Speed";
  static constexpr char kAlphaLimitLabel[] = "Alpha Limit";
  static constexpr char kWingsLabel[] = "Wings";
  static constexpr char kMainWing[] = "Main Wing";

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
  st::Range<double> alphaLimit() const;
  QString requiredWingName() const;
  WingWidget* getWing(const int& index) const;
  WingWidget* getMainWing() const;
  int count() const;

private:
  int index_ = 0;
  ParamGetterWidget_DoubleSpinBox* cruise_speed_;
  ParamGetterWidget_DoubleRange* alpha_limit_;
  qt::TabWidget* tabs_;

  void addWingWidget();
  void addWingWidgetWithName(const std::string& wing_name);
  void removeWingWidget();
  void renameWingWidget(const int index);
};
}  // namespace gb
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
