#pragma once

#include <QTreeWidget>

#include "./rviz.hpp"

namespace gui
{
namespace setup_assistant
{
class FrameTreeWidget : public QTreeWidget
{
  Q_OBJECT

  using self = FrameTreeWidget;
  using super = QTreeWidget;

public:
  explicit FrameTreeWidget(const RobotInfo& robot, RvizWidget* rviz);

  void updateInternalDataStructures();

private Q_SLOTS:
  void onItemClicked(QTreeWidgetItem* item, int col);

  /* 文字列の長さに応じて列の幅を調整する． */
  void resizeColumns();

private:
  const RobotInfo& robot_;
  RvizWidget* rviz_;

  void addTreeItemsRec(QTreeWidgetItem* parent_item);
};
}  // namespace setup_assistant
}  // namespace gui
