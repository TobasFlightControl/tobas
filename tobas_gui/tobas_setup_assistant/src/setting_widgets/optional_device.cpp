#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/setting_widgets/optional_device.hpp"

namespace gui
{
namespace setup_assistant
{
void OptionalDeviceWidget::initialize()
{
  addTitleAndDescription();

  equipped_ = new QCheckBox(QString(name()) + " Equipped");
  equipped_->setFont(qt::DefaultFont(kBodyPSize));
  equipped_->setChecked(defaultEquipped());
  connect(equipped_, &QCheckBox::toggled, this, &self::onEquippedToggled);
  rows_->addWidget(equipped_);

  // Enable, Disableを一括で管理するために，設定ウィジェットを全て1つのウィジェットの子にする．
  config_ = new QWidget();
  config_->setEnabled(defaultEquipped());
  rows_->addWidget(config_);

  param_rows_ = new QVBoxLayout();
  config_->setLayout(param_rows_);

  onInit();

  rows_->addStretch();
}

void OptionalDeviceWidget::onEquippedToggled(bool checked)
{
  config_->setEnabled(checked);
}

bool OptionalDeviceWidget::equipped() const
{
  return equipped_->isChecked();
}
}  // namespace setup_assistant
}  // namespace gui
