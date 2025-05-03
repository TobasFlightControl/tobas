#include <QHBoxLayout>
#include <QDebug>

#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_control_system/status_viewer/status.hpp"

namespace gui
{
namespace gcs
{
StatusWidget::StatusWidget(const QString& text)
{
  led_ = new qt::CircleWidget();
  led_->setFixedSize(kLEDSize, kLEDSize);

  reset();

  const auto cols = new QHBoxLayout();
  cols->addWidget(led_);
  cols->addWidget(new qt::Label(text, kTextPSize));
  setLayout(cols);
}

void StatusWidget::reset()
{
  led_->setColor(kUnknownColor);
}

void StatusWidget::setStatus(status_t status)
{
  switch (status) {
    case PASSED:
      led_->setColor(kPassedColor);
      break;
    case FAILED:
      led_->setColor(kFailedColor);
      break;
    case IGNORED:
      led_->setColor(kIgnoredColor);
      break;
    default:
      qWarning() << "Unknown status: " << status;
      reset();
      break;
  }
}

void StatusWidget::setStatus(uint8_t status)
{
  setStatus(static_cast<status_t>(status));
}

void StatusWidget::setStatus(bool ok)
{
  const auto status = ok ? PASSED : FAILED;
  setStatus(status);
}
}  // namespace gcs
}  // namespace gui
