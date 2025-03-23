#pragma once

#include "tobas_setup_assistant/robot_info.hpp"
#include "./selected_link.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class SelectedLinksWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = SelectedLinksWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;

Q_SIGNALS:
  void linkRemoved(const QString& link_name);

public:
  explicit SelectedLinksWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  void updateInternalDataStructures();

  bool isValid();

  void addLink(const QString& link_name);
  void removeLink(const QString& link_name);

  int numUnits() const;

  QString linkName(int index) const;

  /* タブのインデックスを返す．存在しなければ-1を返す． */
  int index(const QString& link_name) const;

  SelectedLinkWidget* widget(int index);
  const SelectedLinkWidget* widget(int index) const;
  SelectedLinkWidget* widget(const QString& link_name);
  const SelectedLinkWidget* widget(const QString& link_name) const;

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

private Q_SLOTS:
  void onTabCloseRequested(int index);
  void onCopyFromLeftButtonClicked(const QString& link_name);
  void onCopyToAllButtonClicked(const QString& link_name);
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
