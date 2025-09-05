#include "tobas_hardware_setup/base.hpp"

#include <QTimer>

#include <tobas_qt_tools/font.hpp>

#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
BaseHardwareSetupWidget::BaseHardwareSetupWidget()
{
  setBackgroundColor(QPalette::Base);

  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));

  rows_ = new QVBoxLayout();
  rows_->addWidget(title_, 0, Qt::AlignTop);
  rows_->addSpacing(30);

  setLayout(rows_);

  QTimer::singleShot(0, this, &BaseHardwareSetupWidget::initialize);
}

void BaseHardwareSetupWidget::initialize()
{
  title_->setText(title());
}
}  // namespace hw
}  // namespace gui
