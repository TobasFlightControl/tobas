#include "tobas_setup_assistant/setting_tabs/propulsion_system/selected_link.hpp"

namespace gui
{
namespace setup_assistant
{
SelectedLinkWidget::SelectedLinkWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout(this);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  copy_from_left_button_ = new QPushButton("Copy From Left");
  copy_from_left_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(copy_from_left_button_, &QPushButton::clicked, [&]() { Q_EMIT copyFromLeftButtonClicked(); });
  button_cols->addWidget(copy_from_left_button_);

  copy_to_all_button_ = new QPushButton("Copy To All");
  copy_to_all_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(copy_to_all_button_, &QPushButton::clicked, [&]() { Q_EMIT copyToALlButtonClicked(); });
  button_cols->addWidget(copy_to_all_button_);

  button_cols->addStretch();

  tabs_ = new qt::TabWidget();
  tabs_->ignoreWheelEvent();
  tabs_->setSize(kTabWidth, kTabHeight);
  rows->addWidget(tabs_);

  esc_ = new ESCWidget();
  motor_ = new MotorWidget();
  propeller_ = new PropellerWidget();
  electrodynamics_ = new ElectrodynamicsWidget(node, motor_, aerodynamics_);
  aerodynamics_ = new AerodynamicsWidget(node, propeller_);
  speed_limit_ = new SpeedLimitWidget(electrodynamics_, aerodynamics_);

  tabs_->addTab(esc_, esc_->name());
  tabs_->addTab(motor_, motor_->name());
  tabs_->addTab(propeller_, propeller_->name());
  tabs_->addTab(electrodynamics_, electrodynamics_->name());
  tabs_->addTab(aerodynamics_, aerodynamics_->name());
  tabs_->addTab(speed_limit_, speed_limit_->name());

  rows->addStretch();
}

bool SelectedLinkWidget::isValid()
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    if (!widget->isValid())
      return false;
  }

  return true;
}

void SelectedLinkWidget::copyFrom(const SelectedLinkWidget* src)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto des_widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    const auto src_widget = qobject_cast<BaseSelectedLinkSettingWidget*>(src->tabs_->widget(i));
    des_widget->copyFrom(src_widget);
  }
}

YAML::Node SelectedLinkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    node[widget->name()] = widget->dump();
  }

  return node;
}

void SelectedLinkWidget::load(const YAML::Node& node)
{
  for (int i = 0; i < tabs_->count(); ++i)
  {
    const auto widget = qobject_cast<BaseSelectedLinkSettingWidget*>(tabs_->widget(i));
    widget->load(node[widget->name()]);
  }
}

const ESCWidget* SelectedLinkWidget::esc() const
{
  return esc_;
}

const MotorWidget* SelectedLinkWidget::motor() const
{
  return motor_;
}

const PropellerWidget* SelectedLinkWidget::propeller() const
{
  return propeller_;
}

const ElectrodynamicsWidget* SelectedLinkWidget::electrodynamics() const
{
  return electrodynamics_;
}

const AerodynamicsWidget* SelectedLinkWidget::aerodynamics() const
{
  return aerodynamics_;
}

const SpeedLimitWidget* SelectedLinkWidget::speed_limit() const
{
  return speed_limit_;
}
}  // namespace setup_assistant
}  // namespace gui
