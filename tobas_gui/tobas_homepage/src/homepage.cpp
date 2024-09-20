#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_homepage/homepage.hpp"

namespace gui
{
namespace homepage
{
HomepageWidget::HomepageWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new QLabel("Tobas");
  title->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));
  qt::addWidgetCenter(title, rows);

  const auto subtitle = new QLabel("— The Flight Controller for All Drones —");
  subtitle->setFont(qt::DefaultFont(kSubtitlePSize, QFont::Bold));
  qt::addWidgetCenter(subtitle, rows);

  // TODO: ニュース，リリースノート，関連リンクなど

  rows->addStretch();
}
}  // namespace homepage
}  // namespace gui
