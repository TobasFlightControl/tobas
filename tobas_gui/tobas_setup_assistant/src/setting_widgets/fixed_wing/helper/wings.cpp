#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/wings.hpp>

#include <QVBoxLayout>
#include <QToolButton>
#include <QInputDialog>

#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/wing.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
WingsWidget::WingsWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::TabWidget();
  rows->addWidget(tabs_);

  const auto wing = new WingWidget();
  tabs_->addTab(wing, "main_wing");

  // "+" and "-" buttons
  const auto buttons = new QWidget(tabs_);
  const auto button_layout = new QHBoxLayout(buttons);
  button_layout->setContentsMargins(0, 0, 0, 0);
  button_layout->setSpacing(0);

  const auto add_button = new QToolButton(buttons);
  add_button->setText("+");

  const auto remove_button = new QToolButton(buttons);
  remove_button->setText("-");

  button_layout->addWidget(add_button);
  button_layout->addWidget(remove_button);

  tabs_->setCornerWidget(buttons, Qt::TopRightCorner);

  connect(add_button, &QToolButton::clicked,
          this, &self::addWingWidget);

  connect(remove_button, &QToolButton::clicked,
          this, &self::removeWingWidget);
  connect(tabs_->tabBar(), &QTabBar::tabBarDoubleClicked,
          this, &self::renameWingWidget);
}

void WingsWidget::updateInternalDataStructures()
{
  const auto all_tabs = tabs_->count();
  for (int i = 0; i < all_tabs; i++) {
    const auto widget = static_cast<WingWidget*>(tabs_->widget(i));
    widget->updateInternalDataStructures();
  }
}

void WingsWidget::setToDefaults()
{
  const auto all_tabs = tabs_->count();
  for (int i = 0; i < all_tabs; i++) {
    const auto widget = static_cast<WingWidget*>(tabs_->widget(i));
    widget->setToDefaults();
  }
}

bool WingsWidget::isValid()
{
  bool is_valid = true;
  const auto all_tabs = tabs_->count();
  for (int i = 0; i < all_tabs; i++) {
    const auto widget = static_cast<WingWidget*>(tabs_->widget(i));
    if (!widget->isValid()) {
      is_valid = false;
    }
  }
  return is_valid;
}

void WingsWidget::addWingWidget()
{
  index_++;

  const auto wing = new WingWidget();
  wing->setToDefaults();

  tabs_->addTab(wing, QString("wing_%1").arg(index_));
  tabs_->setCurrentWidget(wing);
}

void WingsWidget::removeWingWidget()
{
  const int current_index = tabs_->currentIndex();

  if (current_index < 0) {
    return;
  }

  QWidget* widget = tabs_->widget(current_index);

  tabs_->removeTab(current_index);
  widget->deleteLater();
}

void WingsWidget::renameWingWidget(const int index)
{
  if (index < 0) {
    return;
  }

  const QString old_name = tabs_->tabText(index);

  bool ok = false;
  const QString new_name =
      QInputDialog::getText(
          this,
          "Rename Wing",
          "Name:",
          QLineEdit::Normal,
          old_name,
          &ok);

  if (ok && !new_name.trimmed().isEmpty()) {
    tabs_->setTabText(index, new_name.trimmed());
  }
}

}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
