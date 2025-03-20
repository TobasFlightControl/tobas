#include <QVBoxLayout>
#include <QTimer>

#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/base.hpp"
#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_Base::AerodynamicsWidget_Base()
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

  QTimer::singleShot(0, this, &AerodynamicsWidget_Base::initialize);
}

void AerodynamicsWidget_Base::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void AerodynamicsWidget_Base::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}

void AerodynamicsWidget_Base::initialize()
{
  description_->setText(description());
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
