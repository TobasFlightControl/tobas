#include <QHBoxLayout>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_control_system/status_viewer/status.hpp"

namespace gui
{
namespace control_system
{
StatusWidget::StatusWidget(const QString& text)
{
  led_ = new qt::CircleWidget();
  led_->setFixedSize(kLEDSize, kLEDSize);

  const auto cols = new QHBoxLayout();
  cols->addWidget(led_);
  cols->addWidget(new qt::Label(text, kTextPSize));

  setLayout(cols);

  reset();
}

void StatusWidget::setStatus(bool status)
{
  if (status)
    led_->setColor(Qt::green);
  else
    led_->setColor(Qt::red);
}

void StatusWidget::reset()
{
  led_->setColor(Qt::gray);
}
}  // namespace control_system
}  // namespace gui
