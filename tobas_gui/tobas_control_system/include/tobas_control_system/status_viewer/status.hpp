#pragma once

#include <tobas_qt_tools/widgets/circle_widget.hpp>

namespace gui
{
namespace gcs
{
class StatusWidget : public QWidget
{
  Q_OBJECT

  static constexpr int kLEDSize = 20;
  static constexpr int kTextPSize = 12;

  static constexpr auto kPassedColor = Qt::green;
  static constexpr auto kFailedColor = Qt::red;
  static constexpr auto kIgnoredColor = Qt::yellow;
  static constexpr auto kUnknownColor = Qt::gray;

public:
  enum status_t : uint8_t
  {
    PASSED = 0,
    FAILED = 1,
    IGNORED = 2,
  };

  explicit StatusWidget(const QString& text);

  void reset();

  void setStatus(status_t status);
  void setStatus(uint8_t status);
  void setStatus(bool ok);

private:
  qt::CircleWidget* led_;
};
}  // namespace gcs
}  // namespace gui
