#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>

namespace gui
{
namespace log
{
class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

public:
  explicit FlightLogsWidgetGCS();

  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onDeleteButtonClicked(const QString& log_name);
};
}  // namespace log
}  // namespace gui
