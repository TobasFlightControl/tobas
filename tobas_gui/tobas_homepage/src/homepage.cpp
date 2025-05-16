#include "tobas_homepage/homepage.hpp"

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace gui
{
namespace homepage
{
HomepageWidget::HomepageWidget()
{
  // TODO: ニュース，リリースノート，関連リンクなど

  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Tobas", kTitlePSize, QFont::Bold), rows);
  qt::addWidgetCenter(new qt::Label("— The Flight Controller for All Drones —", kSubtitlePSize, QFont::Bold), rows);
  rows->addStretch();

  setLayout(rows);
}
}  // namespace homepage
}  // namespace gui
