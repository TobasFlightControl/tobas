#include "tobas_bootmedia_config/base.hpp"

#include <QTimer>

#include <tobas_qt_tools/font.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
BaseConfigWidget::BaseConfigWidget()
{
  // QTabWidgetのデフォルトの背景色は白すぎるため，ベース色に固定．
  setAutoFillBackground(true);
  auto pal = palette();
  pal.setColor(QPalette::Window, pal.color(QPalette::Base));

  title_ = new QLabel();
  title_->setFont(qt::DefaultFont(kTitlePSize, QFont::Bold));

  rows_ = new QVBoxLayout();
  rows_->addWidget(title_, 0, Qt::AlignTop);
  rows_->addSpacing(30);

  setLayout(rows_);

  QTimer::singleShot(0, this, &BaseConfigWidget::initialize);
}

void BaseConfigWidget::initialize()
{
  title_->setText(title());
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
