#pragma once

#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace gui
{
namespace homepage
{
class HomepageWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = HomepageWidget;
  using super = qt::ScrollArea;

  static constexpr int kTitlePSize = 50;
  static constexpr int kSubtitlePSize = 20;

public:
  explicit HomepageWidget();
};
}  // namespace homepage
}  // namespace gui
