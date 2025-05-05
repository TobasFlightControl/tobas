#include "tobas_setup_assistant/setting_tabs/optional_device.hpp"

#include <QTimer>

#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/font.hpp>

namespace gui
{
namespace sa
{
OptionalDeviceWidget::OptionalDeviceWidget()
{
  equipped_ = new QCheckBox();
  equipped_->setFont(qt::DefaultFont(kBodyPSize));
  connect(equipped_, &QCheckBox::toggled, this, &self::onEquippedToggled);
  addWidget(equipped_);

  // Enable, Disableを一括で管理するために，設定ウィジェットを全て1つのウィジェットの子にする．
  config_ = new QWidget();
  addWidget(config_);

  param_rows_ = new QVBoxLayout();
  config_->setLayout(param_rows_);

  QTimer::singleShot(0, this, &self::initialize);
}

bool OptionalDeviceWidget::equipped() const
{
  return equipped_->isChecked();
}

void OptionalDeviceWidget::initialize()
{
  equipped_->setText(QString(name()) + " Equipped");
  equipped_->setChecked(defaultEquipped());
  config_->setEnabled(defaultEquipped());
}

void OptionalDeviceWidget::onEquippedToggled(bool checked)
{
  config_->setEnabled(checked);
}
}  // namespace sa
}  // namespace gui
