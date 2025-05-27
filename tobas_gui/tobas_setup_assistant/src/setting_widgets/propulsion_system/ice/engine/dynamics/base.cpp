#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/dynamics/base.hpp"

#include <QTimer>
#include <QVBoxLayout>

#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineDynamicsWidget_Base::EngineDynamicsWidget_Base()
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

  QTimer::singleShot(0, this, &EngineDynamicsWidget_Base::initialize);
}

void EngineDynamicsWidget_Base::addWidget(QWidget* widget)
{
  content_rows_->addWidget(widget);
}

void EngineDynamicsWidget_Base::addLayout(QLayout* layout)
{
  content_rows_->addLayout(layout);
}

void EngineDynamicsWidget_Base::initialize()
{
  description_->setText(description());
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
