#include <QVBoxLayout>
#include <QTimer>

#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/base.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
ElectrodynamicsWidget_Base::ElectrodynamicsWidget_Base()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  header_rows_ = new QVBoxLayout();
  rows->addLayout(header_rows_);

  description_ = new qt::DescriptionWidget("", kBodyPSize);
  header_rows_->addWidget(description_);

  content_rows_ = new QVBoxLayout();
  rows->addLayout(content_rows_);

  rows->addStretch();

  QTimer::singleShot(0, this, &ElectrodynamicsWidget_Base::initialize);
}

void ElectrodynamicsWidget_Base::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void ElectrodynamicsWidget_Base::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}

void ElectrodynamicsWidget_Base::initialize()
{
  description_->setText(description());
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
