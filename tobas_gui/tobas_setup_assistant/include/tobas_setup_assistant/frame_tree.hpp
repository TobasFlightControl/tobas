#pragma once

#include <QTreeWidget>

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class FrameTreeWidget : public QTreeWidget
{
  Q_OBJECT

  using super = QTreeWidget;

  static constexpr int kWidth = 200;

public:
  explicit FrameTreeWidget(SetupAssistant* main);

  void updateInternalDataStructures();

private Q_SLOTS:
  void onItemClicked(QTreeWidgetItem* item, int col);

  /* 文字列の長さに応じて列の幅を調整する． */
  void resizeColumns();

private:
  SetupAssistant* main_;

  void addTreeItemsRec(QTreeWidgetItem* parent_item);
};
}  // namespace setup_assistant
}  // namespace gui
