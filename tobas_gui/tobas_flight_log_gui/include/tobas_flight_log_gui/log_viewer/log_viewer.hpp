#pragma once

#include <QWidget>

namespace gui
{
namespace log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;
  using super = QWidget;

public:
  explicit FlightLogViewerWidget();
};
}  // namespace log
}  // namespace gui
