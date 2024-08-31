#pragma once

#include <QLabel>
#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "../../robot_info.hpp"

namespace gui
{
namespace setup_assistant
{
class AvailableLinkItemWidget : public QWidget
{
  Q_OBJECT

  using self = AvailableLinkItemWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 60;
  static constexpr int kButtonHeight = 20;

Q_SIGNALS:
  void addButtonClicked(const QString& link_name);

public:
  explicit AvailableLinkItemWidget(const QString& link_name);

  QString linkName() const;

private:
  void onAddButtonClicked();

private:
  QLabel* link_label_;
  QPushButton* add_button_;
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

  void add(const QString& link_name);
  void remove(const QString& link_name);

private Q_SLOTS:
  void onAddButtonClicked(const QString& link_name);

private:
  const RobotInfo& robot_;
};
}  // namespace setup_assistant
}  // namespace gui
