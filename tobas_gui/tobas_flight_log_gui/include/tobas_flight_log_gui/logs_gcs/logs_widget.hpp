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

public:
  explicit FlightLogsWidgetGCS();

private:
  QPushButton* read_button_;
  QPushButton* delete_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  void read();

private Q_SLOTS:
  void onReadButtonClicked();
  void onDeleteButtonClicked();
  void onCleanButtonClicked();
};
}  // namespace log
}  // namespace gui
