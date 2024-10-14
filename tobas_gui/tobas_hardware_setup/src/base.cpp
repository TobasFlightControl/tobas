#include <QTimer>

#include <tobas_qt_tools/font.hpp>

#include "tobas_hardware_setup/base.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
BaseHardwareSetupWidget::BaseHardwareSetupWidget()
{
  rows_ = new QVBoxLayout();
  setLayout(rows_);

  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  rows_->addWidget(title_, 0, Qt::AlignTop);

  rows_->addSpacing(50);

  QTimer::singleShot(0, this, &BaseHardwareSetupWidget::initialize);
}

void BaseHardwareSetupWidget::initialize()
{
  title_->setText(title());
}
}  // namespace hardware_setup
}  // namespace gui
