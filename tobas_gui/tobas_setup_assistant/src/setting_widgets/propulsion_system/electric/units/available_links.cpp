#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/available_links.hpp"
#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AvailableLinkItemWidget::AvailableLinkItemWidget(const QString& link_name)
{
  link_name_ = new QLabel(link_name);
  link_name_->setFont(qt::DefaultFont(kBodyPSize));

  add_button_ = new QPushButton("Add");
  add_button_->setFixedWidth(kButtonWidth);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(link_name_);
  cols->addWidget(add_button_);
  setLayout(cols);

  // Connection
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
}

QString AvailableLinkItemWidget::linkName() const
{
  return link_name_->text();
}

void AvailableLinkItemWidget::onAddButtonClicked()
{
  Q_EMIT addButtonClicked(linkName());
}

AvailableLinksWidget::AvailableLinksWidget(const RobotInfo& robot) : robot_(robot)
{
  setFixedHeight(kHeight);
  setSelectionMode(QListWidget::NoSelection);
}

void AvailableLinksWidget::updateInternalDataStructures()
{
  clear();

  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    const auto& joint = elem.segment.joint();

    // 回転関節をもつことを確認
    if (joint.type != kdl::Joint::ROTATION)
      continue;

    // エンドリンクであることを確認
    if (!robot_.tree().isEndSegment(link_name))
      continue;

    // リンク名をリストに追加
    addLink(QString::fromStdString(link_name));
  }

  sortItems();
}

bool AvailableLinksWidget::isValid()
{
  return true;
}

void AvailableLinksWidget::addLink(const QString& link_name)
{
  TOBAS_CHECK(robot_.tree().hasSegment(link_name.toStdString()));

  const auto list_item = new qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kListItemHeight));  // 横幅が小さすぎる場合は自動で引き伸ばされる
  list_item->setData(Qt::UserRole, link_name);        // リンク名をソート基準にする
  addItem(list_item);

  const auto widget = new AvailableLinkItemWidget(link_name);
  connect(widget, &AvailableLinkItemWidget::addButtonClicked, this, &self::onAddButtonClicked);
  setItemWidget(list_item, widget);
}

void AvailableLinksWidget::removeLink(const QString& link_name)
{
  const auto link_item = findLink(link_name);
  remove(link_item);
  Q_EMIT linkRemoved(link_name);
}

QListWidgetItem* AvailableLinksWidget::findLink(const QString& link_name)
{
  for (int row = 0; row < count(); ++row)
  {
    const auto link_item = item(row);
    const auto link_widget = qt::qConstPointerCast<AvailableLinkItemWidget>(itemWidget(link_item));

    if (link_widget->linkName() == link_name)
      return link_item;
  }

  throw std::runtime_error("Failed to find " + link_name.toStdString() + " in the available link list.");
}

void AvailableLinksWidget::onAddButtonClicked(const QString& link_name)
{
  removeLink(link_name);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
