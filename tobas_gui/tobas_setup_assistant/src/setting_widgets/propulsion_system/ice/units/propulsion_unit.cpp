#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propulsion_unit.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
PropulsionUnitWidget::PropulsionUnitWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  copy_to_all_btn_ = new QPushButton("Copy To All");
  copy_to_all_btn_->setFixedSize(kButtonWidth, kButtonHeight);
  tobas::qt::addWidgetCenter(copy_to_all_btn_, rows);

  button_cols->addStretch();

  tabs_ = new tobas::qt::TabWidget();
  tabs_->enableWheelEvent(false);
  tabs_->setTabSize(kTabWidth, kTabHeight);
  rows->addWidget(tabs_);

  transmission_ = new TransmissionWidget();
  propeller_ = new PropellerWidget();
  aerodynamics_ = new AerodynamicsWidget(node, propeller_);

  tabs_->addTab(transmission_, transmission_->name());
  tabs_->addTab(propeller_, propeller_->name());
  tabs_->addTab(aerodynamics_, aerodynamics_->name());

  // Connection
  connect(copy_to_all_btn_, &QPushButton::clicked, [this]() { Q_EMIT copyToAllButtonClicked(); });
}

bool PropulsionUnitWidget::isValid()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = tobas::qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    if (!widget->isValid()) {
      return false;
    }
  }

  return true;
}

void PropulsionUnitWidget::copyFrom(const PropulsionUnitWidget* src)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto des_widget = tobas::qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    const auto src_widget = tobas::qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(src->tabs_->widget(i));
    des_widget->copyFrom(src_widget);
  }
}

YAML::Node PropulsionUnitWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = tobas::qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    node[widget->name()] = widget->dump();
  }

  return node;
}

void PropulsionUnitWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = tobas::qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    widget->load(node[widget->name()]);
  }
}

const TransmissionWidget* PropulsionUnitWidget::transmission() const
{
  return transmission_;
}

const PropellerWidget* PropulsionUnitWidget::propeller() const
{
  return propeller_;
}

const AerodynamicsWidget* PropulsionUnitWidget::aerodynamics() const
{
  return aerodynamics_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
