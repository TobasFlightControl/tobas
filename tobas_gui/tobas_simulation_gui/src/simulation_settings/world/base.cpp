#include "tobas_simulation_gui/simulation_settings/world/base.hpp"

namespace gui
{
namespace sim
{
WorldWidget_Base::WorldWidget_Base(const QString& label)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);

  radio_button = new QRadioButton(label);
  cols_->addWidget(radio_button);

  connect(radio_button, &QRadioButton::toggled, this, &WorldWidget_Base::onRadioButtonToggled);
}

bool WorldWidget_Base::isChecked() const
{
  return radio_button->isChecked();
}

void WorldWidget_Base::setChecked(bool checked)
{
  radio_button->setChecked(checked);
}

void WorldWidget_Base::onRadioButtonToggled(bool checked)
{
  setContentsEnabled(checked);
}
}  // namespace sim
}  // namespace gui
