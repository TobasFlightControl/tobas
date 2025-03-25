#include <QTabBar>
#include <QWheelEvent>
#include <QStylePainter>
#include <QStyleOptionTab>

#include "tobas_qt_tools/widgets/vertical_tab_widget.hpp"

namespace qt
{
QSize VerticalTabBar::tabSizeHint(int index) const
{
  auto s = super::tabSizeHint(index);
  if (s.width() < s.height())
    s.transpose();
  s.scale(s.width() * 2, s.height() * 2, Qt::KeepAspectRatio);
  return s;
}

void VerticalTabBar::paintEvent(QPaintEvent*)
{
  QStylePainter painter(this);
  QStyleOptionTab style_option;

  for (int i = 0; i < count(); ++i)
  {
    initStyleOption(&style_option, i);
    painter.drawControl(QStyle::CE_TabBarTabShape, style_option);
    painter.save();

    auto s = style_option.rect.size();
    s.scale(s.width() * 2, s.height() * 2, Qt::KeepAspectRatio);
    QRect rect(QPoint(), s);
    rect.moveCenter(style_option.rect.center());
    style_option.rect = rect;

    const auto center = tabRect(i).center();
    painter.translate(center);
    painter.rotate(90);
    painter.translate(center * -1);
    painter.drawControl(QStyle::CE_TabBarTabLabel, style_option);
    painter.restore();
  }
}

VerticalTabWidget::VerticalTabWidget(QWidget* parent) : super(parent)
{
  setTabBar(new VerticalTabBar());
  setTabPosition(QTabWidget::West);
}
}  // namespace qt
