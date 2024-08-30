#include <QVBoxLayout>

#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/propeller.hpp"

namespace gui
{
namespace setup_assistant
{
PropellerWidget::PropellerWidget()
{
  auto rows = new QVBoxLayout(this);

  num_blade_ = new ParamGetterWidget_SpinBox("Number of blades", "Number of blades per propeller.");
  num_blade_->setMinimum(1);
  num_blade_->setValue(2);
  rows->addWidget(num_blade_);

  diameter_ = new ParamGetterWidget_SpinBox("Propeller Diameter", "Diameter of the propeller's rotational plane.");
  diameter_->setMinimum(1);
  diameter_->setValue(10);
  diameter_->setSuffix(" inch");
  rows->addWidget(diameter_);

  blade_chord_ =
    new ParamGetterWidget_SpinBox("75% Blade chord", "Chord length at 75% of the distance from the blade's center.");
  blade_chord_->setMinimum(1);
  blade_chord_->setValue(15);
  blade_chord_->setSuffix(" mm");
  rows->addWidget(blade_chord_);

  pitch_angle_ = new ParamGetterWidget_SpinBox(
    "75% Blade pitch angle", "Twist angle at 75% of the distance from the blade's center.");
  pitch_angle_->setMinimum(1);
  pitch_angle_->setMaximum(90);
  pitch_angle_->setValue(15);
  pitch_angle_->setSuffix(" deg");
  rows->addWidget(pitch_angle_);

  rows->addStretch();
}

const char* PropellerWidget::name()
{
  return "Propeller";
}

bool PropellerWidget::isValid()
{
  return true;
}

void PropellerWidget::copyFrom(const PropellerWidget* src)
{
  num_blade_->setValue(src->num_blade_->getValue());
  diameter_->setValue(src->diameter_->getValue());
  blade_chord_->setValue(src->blade_chord_->getValue());
  pitch_angle_->setValue(src->pitch_angle_->getValue());
}

YAML::Node PropellerWidget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[num_blade_->name()] = num_blade_->getValue();
  node[diameter_->name()] = diameter_->getValue();
  node[blade_chord_->name()] = blade_chord_->getValue();
  node[pitch_angle_->name()] = pitch_angle_->getValue();

  return node;
}

void PropellerWidget::load(const YAML::Node& node)
{
  num_blade_->setValue(node[num_blade_->name()].as<int>());
  diameter_->setValue(node[diameter_->name()].as<int>());
  blade_chord_->setValue(node[blade_chord_->name()].as<int>());
  pitch_angle_->setValue(node[pitch_angle_->name()].as<int>());
}

int PropellerWidget::numBlade() const
{
  return num_blade_->getValue();
}

double PropellerWidget::diameter() const
{
  return tobas_std::inch2meter(diameter_->getValue());
}

double PropellerWidget::radius() const
{
  return diameter() / 2;
}

double PropellerWidget::bladeChord() const
{
  return blade_chord_->getValue() * 1e-3;
}

double PropellerWidget::pitchAngle() const
{
  return tobas_std::deg2rad(pitch_angle_->getValue());
}
}  // namespace setup_assistant
}  // namespace gui
