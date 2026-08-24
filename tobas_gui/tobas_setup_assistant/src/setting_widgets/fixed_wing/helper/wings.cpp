#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/wings.hpp>

#include <QInputDialog>
#include <QToolButton>
#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
WingsWidget::WingsWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  cruise_speed_ = new ParamGetterWidget_DoubleSpinBox("Cruise Speed");
  cruise_speed_->setDecimals(2);
  cruise_speed_->setSuffix(" m/s");
  rows->addWidget(cruise_speed_);

  tabs_ = new qt::TabWidget();
  tabs_->tabBar()->setElideMode(Qt::ElideRight);
  rows->addWidget(tabs_);

  const auto wing = new WingWidget();
  tabs_->addTab(wing, kMainWing);

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

  tabs_->setCornerWidget(buttons, Qt::TopLeftCorner);

  connect(add_button, &QToolButton::clicked, this, &self::addWingWidget);

  connect(remove_button, &QToolButton::clicked, this, &self::removeWingWidget);
  connect(tabs_->tabBar(), &QTabBar::tabBarDoubleClicked, this, &self::renameWingWidget);
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
  cruise_speed_->setValue(15.0);

  const auto all_tabs = tabs_->count();
  for (int i = 0; i < all_tabs; i++) {
    const auto widget = static_cast<WingWidget*>(tabs_->widget(i));
    widget->setToDefaults();
  }
}

bool WingsWidget::isValid() const
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

YAML::Node WingsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto all_tabs = tabs_->count();
  for (int i = 0; i < all_tabs; i++) {
    node[tabs_->tabText(i).toStdString()] = getWing(i)->dump();
  }

  return node;
}

void WingsWidget::load(const YAML::Node& node)
{
  int i = 0;
  for (const auto& item : node) {
    const auto wing_name = item.first.as<std::string>();
    if (wing_name != kMainWing) {
      addWingWidgetWithName(wing_name);
    }
    const auto widget = static_cast<WingWidget*>(tabs_->widget(i));
    widget->load(item.second);
    i++;
  }

  tabs_->setCurrentIndex(0); // main wing widgetに戻す
}

double WingsWidget::cruiseSpeed() const
{
  return cruise_speed_->getValue();
}

QString WingsWidget::requiredWingName() const
{
  return kMainWing;
}

WingWidget* WingsWidget::getWing(const int& index) const
{
  return static_cast<WingWidget*>(tabs_->widget(index));
}

WingWidget* WingsWidget::getMainWing() const
{
  return getWing(0);
}

int WingsWidget::count() const
{
  return tabs_->count();
}

void WingsWidget::addWingWidget()
{
  index_++;

  const auto wing = new WingWidget();
  wing->setToDefaults();

  QString default_name = QString("wing_%1").arg(index_);
  tabs_->addTab(wing, default_name);
  tabs_->setCurrentWidget(wing);
  Q_EMIT tabAdded(default_name);
}

void WingsWidget::addWingWidgetWithName(const std::string& wing_name)
{
  index_++;

  const auto wing = new WingWidget();
  wing->setToDefaults();

  tabs_->addTab(wing, QString::fromStdString(wing_name));
  tabs_->setCurrentWidget(wing);
  Q_EMIT tabAdded(QString::fromStdString(wing_name));
}

void WingsWidget::removeWingWidget()
{
  const int current_index = tabs_->currentIndex();

  if (current_index < 0) {
    return;
  }

  // main_wingは消去しない. 全てのtabを消去してしまうとtab部の+-ボタンまで一緒に消えてしまうのも理由の一つ
  if (tabs_->tabText(current_index) == kMainWing) {
    return;
  }

  QWidget* widget = tabs_->widget(current_index);

  tabs_->removeTab(current_index);
  widget->deleteLater();
  Q_EMIT tabRemoved(current_index);
}

void WingsWidget::renameWingWidget(const int index)
{
  if (index < 0) {
    return;
  }

  const QString old_name = tabs_->tabText(index);

  bool ok = false;
  const QString new_name = QInputDialog::getText(this, "Rename Wing", "New Name:", QLineEdit::Normal, old_name, &ok);

  if (!ok) {
    return;
  }

  const QString name = new_name.trimmed();
  if (name.isEmpty()) {
    return;
  }

  bool duplicate = false;
  for (int i = 0; i < tabs_->count(); ++i) {
    if (i != index && tabs_->tabText(i) == name) {
      duplicate = true;
      break;
    }
  }
  if (duplicate) {
    qt::qWarnBox(this, "The name is already used by another wing.");
    return;
  }

  tabs_->setTabText(index, name);
  Q_EMIT tabRenamed(index, name);
}
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
