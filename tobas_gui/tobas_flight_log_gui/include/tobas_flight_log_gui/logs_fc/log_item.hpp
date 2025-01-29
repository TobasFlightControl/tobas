#pragma once

#include <QLabel>
#include <QPushButton>

namespace gui
{
namespace log
{
class FlightLogItemWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetFC;
  using super = QWidget;

  static constexpr int kButtonWidth = 80;

Q_SIGNALS:
  void downloadButtonClicked(const QString& log_name);
  void deleteButtonClicked(const QString& log_name);

public:
  explicit FlightLogItemWidgetFC(const QString& log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QPushButton* download_button_;
  QPushButton* delete_button_;

private Q_SLOTS:
  void onDownloadButtonClicked();
  void onDeleteButtonClicked();
};
}  // namespace log
}  // namespace gui
