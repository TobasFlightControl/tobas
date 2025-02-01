#pragma once

#include <QLabel>
#include <QPushButton>

namespace gui
{
namespace log
{
class FlightLogItemWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogItemWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 80;

Q_SIGNALS:
  void deleteButtonClicked(const QString& log_name);

public:
  explicit FlightLogItemWidgetGCS(const QString& log_name);

  QString logName() const;

private:
  QLabel* log_name_;
  QPushButton* delete_button_;

private Q_SLOTS:
  void onDeleteButtonClicked();
};
}  // namespace log
}  // namespace gui
