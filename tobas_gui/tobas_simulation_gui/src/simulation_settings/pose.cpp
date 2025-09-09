#include "tobas_simulation_gui/simulation_settings/pose.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace sim
{
PoseWidget::PoseWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addWidget(new qt::Label("Initial Pose", cmn::kLabelPSize, QFont::Bold));

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  const auto xyz_form = new qt::FormLayout();
  cols->addLayout(xyz_form, 1);
  static constexpr std::array<const char*, 3> kLabelsXYZ = { "X:", "Y:", "Z:" };
  for (size_t i = 0; i < 3; ++i) {
    xyz_[i] = new qt::DoubleSpinBox();
    xyz_[i]->setDecimals(1);
    xyz_[i]->setSingleStep(0.1);
    xyz_[i]->setSuffix(" m");

    xyz_form->addRow(kLabelsXYZ[i], xyz_[i]);
  }

  const auto rpy_form = new qt::FormLayout();
  cols->addLayout(rpy_form, 1);
  static constexpr std::array<const char*, 3> kLabelsRPY = { "Roll:", "Pitch:", "Yaw:" };
  for (size_t i = 0; i < 3; ++i) {
    rpy_[i] = new qt::SpinBox();
    rpy_[i]->setSuffix(" deg");
    rpy_[i]->setMinimum(-180);
    rpy_[i]->setMaximum(+180);

    rpy_form->addRow(kLabelsRPY[i], rpy_[i]);
  }

  reset();
}

void PoseWidget::reset()
{
  for (size_t i = 0; i < 3; ++i) {
    xyz_[i]->setValue(kDefaultXYZ[i]);
    rpy_[i]->setValue(kDefaultRPY[i]);
  }
}

double PoseWidget::x() const
{
  return xyz_[0]->value();
}

double PoseWidget::y() const
{
  return xyz_[1]->value();
}

double PoseWidget::z() const
{
  return xyz_[2]->value();
}

double PoseWidget::roll() const
{
  return tobas_std::deg2rad(rpy_[0]->value());
}

double PoseWidget::pitch() const
{
  return tobas_std::deg2rad(rpy_[1]->value());
}

double PoseWidget::yaw() const
{
  return tobas_std::deg2rad(rpy_[2]->value());
}
}  // namespace sim
}  // namespace gui
