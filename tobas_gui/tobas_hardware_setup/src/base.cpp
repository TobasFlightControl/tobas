#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

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
  setLayout(rows_);

  const auto title_label = new QLabel(title());
  title_label->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  title_label->setAlignment(Qt::AlignTop);
  rows_->addWidget(title_label);

  rows_->addSpacing(50);

  onInit();
}
}  // namespace hardware_setup
}  // namespace gui
