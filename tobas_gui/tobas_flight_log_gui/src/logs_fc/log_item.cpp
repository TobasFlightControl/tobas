#include <QHBoxLayout>

#include <tobas_qt_tools/font.hpp>

#include "tobas_flight_log_gui/logs_fc/log_item.hpp"
#include "tobas_flight_log_gui/constants.hpp"

namespace gui
{
namespace log
{
FlightLogItemWidgetFC::FlightLogItemWidgetFC(const QString& log_name)
{
  log_name_ = new QLabel(log_name);
  log_name_->setFont(qt::DefaultFont(kBodyPSize));
  log_name_->setAlignment(Qt::AlignLeft);

  download_button_ = new QPushButton("Download");
  download_button_->setFixedWidth(kButtonWidth);

  delete_button_ = new QPushButton("Delete");
  delete_button_->setFixedWidth(kButtonWidth);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(log_name_);
  cols->addWidget(download_button_);
  cols->addWidget(delete_button_);
  setLayout(cols);

  // Connection
  connect(download_button_, &QPushButton::clicked, this, &self::onDownloadButtonClicked);
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);
}

QString FlightLogItemWidgetFC::logName() const
{
  return log_name_->text();
}

void FlightLogItemWidgetFC::onDownloadButtonClicked()
{
  Q_EMIT downloadButtonClicked(logName());
}

void FlightLogItemWidgetFC::onDeleteButtonClicked()
{
  Q_EMIT deleteButtonClicked(logName());
}
}  // namespace log
}  // namespace gui
