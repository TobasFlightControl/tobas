#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_hardware_setup/base.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
BaseHardwareSetupWidget::BaseHardwareSetupWidget()
{
}

void BaseHardwareSetupWidget::initialize()
{
  rows_ = new QVBoxLayout();
  rows_->addWidget(new qt::Label(title(), kTitlePSize, QFont::Bold), 0, Qt::AlignTop);
  rows_->addSpacing(50);

  setLayout(rows_);

  onInit();
}
}  // namespace hardware_setup
}  // namespace gui
