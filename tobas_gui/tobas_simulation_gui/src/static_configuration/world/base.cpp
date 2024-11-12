#include "tobas_simulation_gui/static_configuration/world/base.hpp"

namespace gui
{
namespace sim
{
WorldWidget_Base::WorldWidget_Base(const QString& label)
{
  cols_ = new QHBoxLayout();
  setLayout(cols_);

  checkbox = new QCheckBox(label);
  cols_->addWidget(checkbox);

  connect(checkbox, &QCheckBox::toggled, this, &WorldWidget_Base::onCheckBoxToggled);
}

bool WorldWidget_Base::isChecked() const
{
  return checkbox->isChecked();
}

void WorldWidget_Base::setChecked(bool checked)
{
  checkbox->setChecked(checked);
}
}  // namespace sim
}  // namespace gui
