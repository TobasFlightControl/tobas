#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/stream.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/selected_links.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
SelectedLinksWidget::SelectedLinksWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot)
  : node_(node), robot_(robot)
{
  enableWheelEvent(false);
  setTabSize(kTabWidth, kTabHeight);
  setMovable(true);
  setTabsClosable(true);

  connect(this, &qt::TabWidget::tabCloseRequested, this, &self::onTabCloseRequested);
}

void SelectedLinksWidget::updateInternalDataStructures()
{
  clear();
}

bool SelectedLinksWidget::isValid()
{
  const auto num_rotors = count();

  // 最低1つは登録されていなければならない
  if (num_rotors == 0) {
    qt::qErrorBox(this, "Please register at least one link as a propulsion system.");
    return false;
  }

  // それぞれのタブの設定が有効であることを確認
  for (int i = 0; i < num_rotors; ++i) {
    if (!widget(i)->isValid()) {
      return false;
    }
  }

  // チャンネルが重複していないことを確認
  QSet<int> channels;
  for (int i = 0; i < num_rotors; ++i) {
    const auto channel = widget(i)->hardwareIface()->pwmChannel();
    if (channels.contains(channel)) {
      qt::qErrorBox(this, "PWM channel " + QString::number(channel) + " is duplicated.");
      return false;
    }
    channels.insert(channel);
  }

  return true;
}

void SelectedLinksWidget::addLink(const QString& link_name)
{
  // タブを追加
  const auto link_widget = new SelectedLinkWidget();
  addTab(link_widget, link_name);

  // Connection
  connect(
    link_widget, &SelectedLinkWidget::copyFromLeftButtonClicked, this,
    std::bind(&self::onCopyFromLeftButtonClicked, this, link_name));
  connect(
    link_widget, &SelectedLinkWidget::copyToAllButtonClicked, this,
    std::bind(&self::onCopyToAllButtonClicked, this, link_name));
}

void SelectedLinksWidget::removeLink(const QString& link_name)
{
  // タブを削除
  removeTab(index(link_name));

  // シグナルを発行
  Q_EMIT linkRemoved(link_name);
}

int SelectedLinksWidget::numUnits() const
{
  return count();
}

QString SelectedLinksWidget::linkName(int index) const
{
  return tabText(index);
}

int SelectedLinksWidget::index(const QString& link_name) const
{
  for (int i = 0; i < count(); ++i) {
    if (linkName(i) == link_name) {
      return i;
    }
  }

  qWarning() << link_name << " is not selected as a propulsion system.";
  return -1;
}

SelectedLinkWidget* SelectedLinksWidget::widget(int index)
{
  return qt::qPointerCast<SelectedLinkWidget>(super::widget(index));
}

const SelectedLinkWidget* SelectedLinksWidget::widget(int index) const
{
  return qt::qConstPointerCast<SelectedLinkWidget>(super::widget(index));
}

SelectedLinkWidget* SelectedLinksWidget::widget(const QString& link_name)
{
  return widget(index(link_name));
}

const SelectedLinkWidget* SelectedLinksWidget::widget(const QString& link_name) const
{
  return widget(index(link_name));
}

void SelectedLinksWidget::onTabCloseRequested(int index)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SelectedLinksWidget::onTabCloseRequested(" << index << ")");

  const auto link_name = linkName(index);
  removeLink(link_name);
}

void SelectedLinksWidget::onCopyFromLeftButtonClicked(const QString& link_name)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SelectedLinksWidget::onCopyFromLeftButtonClicked(" << link_name << ")");

  const auto dst_idx = index(link_name);
  const auto src_idx = dst_idx - 1;
  if (src_idx < 0) {
    qt::qWarnBox(this, "There are no tabs on the left side.");
    return;
  }

  const auto dst_widget = widget(dst_idx);
  const auto src_widget = widget(src_idx);
  dst_widget->copyFrom(src_widget);

  qt::qInfoBox(this, "The settings of \"" + linkName(src_idx) + "\" have been copied to \"" + link_name + "\".");
}

void SelectedLinksWidget::onCopyToAllButtonClicked(const QString& link_name)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "SelectedLinksWidget::onCopyToAllButtonClicked(" << link_name << ")");

  const auto src_idx = index(link_name);
  const auto src_widget = widget(src_idx);

  for (int dst_idx = 0; dst_idx < count(); ++dst_idx) {
    if (dst_idx == src_idx) {
      continue;
    }
    const auto dst_widget = widget(dst_idx);
    dst_widget->copyFrom(src_widget);
  }

  qt::qInfoBox(this, "The settings of \"" + link_name + "\" have been copied to all the other selected links.");
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
