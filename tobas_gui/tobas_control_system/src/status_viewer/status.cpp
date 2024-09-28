#include <QLabel>
#include <QHBoxLayout>

#include <tobas_qt_tools/font.hpp>

#include "tobas_control_system/status_viewer/status.hpp"

namespace gui
{
namespace control_system
{
StatusWidget::StatusWidget(const QString& text)
{
  led_ = new qt::CircleWidget();
  led_->setFixedSize(kLEDSize, kLEDSize);

  const auto text_label = new QLabel(text);
  text_label->setFont(qt::DefaultFont(kTextPSize));

  const auto cols = new QHBoxLayout();
  cols->addWidget(led_);
  cols->addWidget(text_label);

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
