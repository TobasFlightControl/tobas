#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/font.hpp>

#include "tobas_flight_log_gui/constants.hpp"

namespace gui
{
namespace log
{
FlightLogItemWidgetGCS::FlightLogItemWidgetGCS(const QString& log_name)
{
  log_name_ = new QLabel(log_name);
  log_name_->setFont(qt::DefaultFont(kPSize3));

  delete_button_ = new QPushButton("Delete");
  delete_button_->setFixedWidth(kButtonWidth);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(log_name_);
  cols->addWidget(delete_button_);
  setLayout(cols);

  // Connection
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);
}

QString FlightLogItemWidgetGCS::logName() const
{
  return log_name_->text();
}

void FlightLogItemWidgetGCS::onDeleteButtonClicked()
{
  Q_EMIT deleteButtonClicked(logName());
}
}  // namespace log
}  // namespace gui
