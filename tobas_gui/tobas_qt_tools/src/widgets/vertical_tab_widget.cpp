#include "tobas_qt_tools/widgets/vertical_tab_widget.hpp"

#include <QStyleOptionTab>
#include <QStylePainter>
#include <QTabBar>
#include <QWheelEvent>

namespace qt
{
QSize VerticalTabBar::tabSizeHint(int index) const
{
  auto s = super::tabSizeHint(index);
  if (s.width() < s.height()) {
    s.transpose();
  }
  s.scale(s.width() * 2, s.height() * 2, Qt::KeepAspectRatio);
  return s;
}

void VerticalTabBar::paintEvent(QPaintEvent*)
{
  QStylePainter painter(this);
  QStyleOptionTab opt;

  for (int i = 0; i < count(); ++i) {
    initStyleOption(&opt, i);

    if (colors_.contains(i)) {
      opt.palette.setColor(QPalette::Button, colors_.value(i));
    }

    painter.drawControl(QStyle::CE_TabBarTabShape, opt);
    painter.save();

    auto s = opt.rect.size();
    s.scale(s.width() * 2, s.height() * 2, Qt::KeepAspectRatio);
    QRect rect(QPoint(), s);
    rect.moveCenter(opt.rect.center());
    opt.rect = rect;

    const auto center = tabRect(i).center();
    painter.translate(center);
    painter.rotate(90);
    painter.translate(-center);

    painter.drawControl(QStyle::CE_TabBarTabLabel, opt);
    painter.restore();
  }
}

VerticalTabWidget::VerticalTabWidget(QWidget* parent) : super(parent)
{
  tab_bar_ = new VerticalTabBar();
  setTabBar(tab_bar_);

  setTabPosition(QTabWidget::West);
}

void VerticalTabWidget::setTabBackgroundColor(int index, const QColor& color)
{
  tab_bar_->setTabBackgroundColor(index, color);
}

void VerticalTabWidget::clearTabBackgroundColor(int index)
{
  tab_bar_->clearTabBackgroundColor(index);
}
}  // namespace qt
