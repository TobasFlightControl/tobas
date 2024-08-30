#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/aerodynamics/base.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
void AerodynamicsWidget_Base::initialize()
{
  auto rows = new QVBoxLayout(this);

  header_rows_ = new QVBoxLayout();
  rows->addLayout(header_rows_);

  auto description_label = new qt::DescriptionWidget(description(), kBodyPSize);
  header_rows_->addWidget(description_label);

  content_rows_ = new QVBoxLayout();
  rows->addLayout(content_rows_);

  rows->addStretch();

  onInit();
}

void AerodynamicsWidget_Base::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void AerodynamicsWidget_Base::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}
}  // namespace setup_assistant
}  // namespace gui
