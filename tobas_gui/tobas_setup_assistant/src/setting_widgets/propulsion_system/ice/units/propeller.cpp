#include <QVBoxLayout>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propeller.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
PropellerWidget::PropellerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  num_blade_ = new ParamGetterWidget_SpinBox("Number of Blades", "");  // TODO
  num_blade_->setMinimum(1);
  num_blade_->setValue(2);
  rows->addWidget(num_blade_);

  pitch_ref_ = new ParamGetterWidget_SpinBox("Pitch Angle Reference", "");  // TODO
  pitch_ref_->setMinimum(-90);
  pitch_ref_->setMaximum(90);
  pitch_ref_->setValue(0);
  pitch_ref_->setSuffix(" deg");
  rows->addWidget(pitch_ref_);

  pitch_limit_ = new ParamGetterWidget_IntRange("Pitch Angle Limit", "");  // TODO
  pitch_limit_->setMinimum(-90);
  pitch_limit_->setMaximum(+90);
  pitch_limit_->setValue({ -6, 6 });
  pitch_limit_->setSuffix(" deg");
  rows->addWidget(pitch_limit_);

  max_pitch_rate_ = new ParamGetterWidget_SpinBox("Max Pitch Angle Rate", "");  // TODO
  max_pitch_rate_->setMinimum(0);
  max_pitch_rate_->setValue(600);
  max_pitch_rate_->setSuffix(" dps");
  rows->addWidget(max_pitch_rate_);

  rows->addStretch();
}

const char* PropellerWidget::name() const
{
  return "Propeller";
}

bool PropellerWidget::isValid()
{
  return true;
}

void PropellerWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<PropellerWidget>(src);

  num_blade_->setValue(derived->num_blade_->getValue());
  pitch_ref_->setValue(derived->pitch_ref_->getValue());
  pitch_limit_->setValue(derived->pitch_limit_->getValue());
  max_pitch_rate_->setValue(derived->max_pitch_rate_->getValue());
}

YAML::Node PropellerWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_blade_->name()] = num_blade_->getValue();
  node[pitch_ref_->name()] = pitch_ref_->getValue();
  node[pitch_limit_->name()] = pitch_limit_->getValue();
  node[max_pitch_rate_->name()] = max_pitch_rate_->getValue();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blade_->setValue(node[num_blade_->name()].as<int>());
  pitch_ref_->setValue(node[pitch_ref_->name()].as<int>());
  pitch_limit_->setValue(node[pitch_limit_->name()].as<tobas_std::Range<int>>());
  max_pitch_rate_->setValue(node[max_pitch_rate_->name()].as<int>());
}

int PropellerWidget::numBlade() const
{
  return num_blade_->getValue();
}

double PropellerWidget::pitchAngleRef() const
{
  return tobas_std::deg2rad(pitch_ref_->getValue());
}

tobas_std::Range<double> PropellerWidget::pitchAngleLimit() const
{
  const auto lower = tobas_std::deg2rad(pitch_limit_->min());
  const auto upper = tobas_std::deg2rad(pitch_limit_->max());
  return { lower, upper };
}

double PropellerWidget::maxPitchAngleRate() const
{
  return tobas_std::deg2rad(max_pitch_rate_->getValue());
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
