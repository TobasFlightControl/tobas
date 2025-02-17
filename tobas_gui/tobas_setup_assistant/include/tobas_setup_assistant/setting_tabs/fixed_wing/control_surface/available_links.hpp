#pragma once

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
class AvailableLinksWidget : public qt::ListWidget
{
  Q_OBJECT

  using self = AvailableLinksWidget;
  using super = qt::ListWidget;

public:
  explicit AvailableLinksWidget(const RobotInfo& robot);

  void updateInternalDataStructures();

  /* 現在選択されているリンク名を返す．存在しない場合は空文字を返す． */
  QString selected() const;

  void add(const QString& link_name);
  void remove(const QString& link_name);

private:
  const RobotInfo& robot_;
};
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
