#include "tobas_simulation_gui/simulation_settings/debug.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace sim
{
DebugWidget::DebugWidget()
{
  user_debug_ = new QCheckBox("User Debug");

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("Debug", common::kLabelPSize, QFont::Bold));
  rows->addWidget(user_debug_);

  setLayout(rows);
}

bool DebugWidget::userDebug() const
{
  return user_debug_->isChecked();
}
}  // namespace sim
}  // namespace gui
