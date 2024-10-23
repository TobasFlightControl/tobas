#include <QPushButton>
#include <QHBoxLayout>

#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/control_surface/add_remove_buttons.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
AddRemoveButtonsWidget::AddRemoveButtonsWidget(
  AvailableLinksWidget* available_links,
  SelectedLinksWidget* selected_links)
  : available_links_(available_links), selected_links_(selected_links)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto add_button = new QPushButton("⬇");
  add_button->setFixedSize(kButtonWidth, kButtonHeight);
  connect(add_button, &QPushButton::clicked, this, &self::onAddButtonClicked);
  cols->addWidget(add_button);

  const auto remove_button = new QPushButton("⬆");
  remove_button->setFixedSize(kButtonWidth, kButtonHeight);
  connect(remove_button, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  cols->addWidget(remove_button);
}

void AddRemoveButtonsWidget::onAddButtonClicked()
{
  const auto selected_link_name = available_links_->selected();
  if (selected_link_name.isEmpty())
  {
    qt::qErrorBox(this, "No link is selected.");
    return;
  }

  available_links_->remove(selected_link_name);
  selected_links_->add(selected_link_name);
}

void AddRemoveButtonsWidget::onRemoveButtonClicked()
{
  const auto selected_link_name = selected_links_->selected();
  if (selected_link_name.isEmpty())
  {
    qt::qErrorBox(this, "No link is selected.");
    return;
  }

  selected_links_->remove(selected_link_name);
  available_links_->add(selected_link_name);
}
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
