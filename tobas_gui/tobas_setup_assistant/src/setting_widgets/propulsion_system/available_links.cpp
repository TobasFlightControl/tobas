#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/available_links.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
AvailableLinkItemWidget::AvailableLinkItemWidget(const QString& link_name)
{
  const auto cols = new QHBoxLayout(this);

  link_label_ = new QLabel(link_name);
  link_label_->setFont(qt::DefaultFont(kBodyPSize));
  link_label_->setAlignment(Qt::AlignLeft);
  cols->addWidget(link_label_);

  add_button_ = new QPushButton("Add");
  add_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  cols->addWidget(add_button_);
}

QString AvailableLinkItemWidget::linkName() const
{
  return link_label_->text();
}

void AvailableLinkItemWidget::onAddButtonClicked()
{
  Q_EMIT addButtonClicked(linkName());
}

AvailableLinksWidget::AvailableLinksWidget(const RobotInfo& robot) : robot_(robot)
{
  setFixedHeight(kHeight);
}

void AvailableLinksWidget::updateInternalDataStructures()
{
  clear();

  for (const auto& [link_name, elem] : robot_.tree().getSegments())
  {
    const auto& joint = elem.segment.joint();

    // 回転関節をもつことを確認
    if (joint.type != kdl::Joint::RotAxis)
      continue;

    // トランスミッションをもたないことを確認
    // TODO: プロペラ専用のトランスミッションを用意する
    if (robot_.hardwareInterface(joint.name) != hw_interface::NONE)
      continue;

    // エンドリンクであることを確認
    if (!robot_.tree().isEndSegment(link_name))
      continue;

    // リンク名をリストに追加
    add(QString::fromStdString(link_name));
  }

  sortItems();
}

bool AvailableLinksWidget::isValid()
{
  return true;
}

void AvailableLinksWidget::add(const QString& link_name)
{
  TOBAS_CHECK(robot_.tree().hasSegment(link_name.toStdString()));
  TOBAS_CHECK(!contains(link_name));

  const auto list_item = new qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kItemHeight));  // 横幅が小さすぎる場合は自動で引き伸ばされる
  list_item->setData(Qt::UserRole, link_name);    // リンク名をソート基準にする
  addItem(list_item);

  const auto widget = new AvailableLinkItemWidget(link_name);
  connect(widget, &AvailableLinkItemWidget::addButtonClicked, this, &self::onAddButtonClicked);
  setItemWidget(list_item, widget);

  sortItems();
}

void AvailableLinksWidget::remove(const QString& link_name)
{
  for (int row = 0; row < count(); ++row)
  {
    const auto link_widget = qobject_cast<AvailableLinkItemWidget*>(itemWidget(item(row)));
    if (link_widget->linkName() == link_name)
    {
      takeItem(row);
      return;
    }
  }

  qt::qErrorBox(this, "Failed to remove link \"" + link_name + "\" from the available link list.");
}

void AvailableLinksWidget::onAddButtonClicked(const QString& link_name)
{
  remove(link_name);
  Q_EMIT linkRemoved(link_name);
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
