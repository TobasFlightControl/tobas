#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/selected_link.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
SelectedLinkWidget::SelectedLinkWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  copy_from_left_button_ = new QPushButton("Copy From Left");
  copy_from_left_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(copy_from_left_button_);

  copy_to_all_button_ = new QPushButton("Copy To All");
  copy_to_all_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(copy_to_all_button_);

  button_cols->addStretch();

  tabs_ = new qt::TabWidget();
  tabs_->ignoreWheelEvent();
  tabs_->setTabSize(kTabWidth, kTabHeight);
  rows->addWidget(tabs_);

  general_ = new GeneralWidget();
  transmission_ = new TransmissionWidget();
  propeller_ = new PropellerWidget();
  hw_iface_ = new VPitchHardwareIfaceWidget();
  aerodynamics_ = new AerodynamicsWidget();

  tabs_->addTab(general_, general_->name());
  tabs_->addTab(transmission_, transmission_->name());
  tabs_->addTab(propeller_, propeller_->name());
  tabs_->addTab(hw_iface_, hw_iface_->name());
  tabs_->addTab(aerodynamics_, aerodynamics_->name());

  rows->addStretch();

  // Connection
  connect(copy_to_all_button_, &QPushButton::clicked, [this]() { Q_EMIT copyToAllButtonClicked(); });
  connect(copy_from_left_button_, &QPushButton::clicked, [this]() { Q_EMIT copyFromLeftButtonClicked(); });
}

bool SelectedLinkWidget::isValid()
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    if (!widget->isValid())
      return false;
  }

  return true;
}

void SelectedLinkWidget::copyFrom(const SelectedLinkWidget* src)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto des_widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    const auto src_widget = qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(src->tabs_->widget(i));
    des_widget->copyFrom(src_widget);
  }
}

YAML::Node SelectedLinkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qt::qConstPointerCast<const BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    node[widget->name()] = widget->dump();
  }

  return node;
}

void SelectedLinkWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qt::qPointerCast<BaseSelectedLinkSettingWidget>(tabs_->widget(i));
    widget->load(node[widget->name()]);
  }
}

const GeneralWidget* SelectedLinkWidget::general() const
{
  return general_;
}

const TransmissionWidget* SelectedLinkWidget::transmission() const
{
  return transmission_;
}

const PropellerWidget* SelectedLinkWidget::propeller() const
{
  return propeller_;
}

const VPitchHardwareIfaceWidget* SelectedLinkWidget::hardwareIface() const
{
  return hw_iface_;
}

const AerodynamicsWidget* SelectedLinkWidget::aerodynamics() const
{
  return aerodynamics_;
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
