#pragma once

#include "./available_links.hpp"
#include "./selected_links.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
class AddRemoveButtonsWidget : public QWidget
{
  Q_OBJECT

  using self = AddRemoveButtonsWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

Q_SIGNALS:
  void linkAdded(const QString& link_name);
  void linkRemoved(const QString& link_name);

public:
  explicit AddRemoveButtonsWidget(AvailableLinksWidget* available_links, SelectedLinksWidget* selected_links);

private Q_SLOTS:
  void onAddButtonClicked();
  void onRemoveButtonClicked();

private:
  AvailableLinksWidget* available_links_;
  SelectedLinksWidget* selected_links_;
};
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
