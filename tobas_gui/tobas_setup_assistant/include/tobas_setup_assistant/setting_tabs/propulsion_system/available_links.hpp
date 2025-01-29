#pragma once

#include <QLabel>
#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "../../robot_info.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
class AvailableLinkItemWidget : public QWidget
{
  Q_OBJECT

  using self = AvailableLinkItemWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 60;

Q_SIGNALS:
  void addButtonClicked(const QString& link_name);

public:
  explicit AvailableLinkItemWidget(const QString& link_name);

  QString linkName() const;

private:
  QLabel* link_label_;
  QPushButton* add_button_;

private Q_SLOTS:
  void onAddButtonClicked();
};

class AvailableLinksWidget : public qt::ListWidget
{
  Q_OBJECT

  using self = AvailableLinksWidget;
  using super = qt::ListWidget;

  static constexpr int kHeight = 200;
  static constexpr int kItemHeight = 40;

Q_SIGNALS:
  void linkRemoved(const QString& link_name);

public:
  explicit AvailableLinksWidget(const RobotInfo& robot);

  void updateInternalDataStructures();

  bool isValid();

  void addLink(const QString& link_name);
  void removeLink(const QString& link_name);

private:
  const RobotInfo& robot_;

  QListWidgetItem* findLink(const QString& link_name);

private Q_SLOTS:
  void onAddButtonClicked(const QString& link_name);
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
