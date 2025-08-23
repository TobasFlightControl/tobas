#include "tobas_bootmedia_config/wifi_client/add_wifi_dialog.hpp"

#include <QFormLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "tobas_bootmedia_config/constants.hpp"

namespace gui
{
namespace bm
{
AddWifiDialog::AddWifiDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add New Network");

  ssid_ = new QLineEdit();
  psk_ = new qt::PasswordEdit();

  priority_ = new qt::SpinBox();
  priority_->setMinimum(0);
  priority_->setMaximum(99);
  priority_->setValue(0);

  btn_box_ = new QDialogButtonBox();
  btn_box_->setOrientation(Qt::Horizontal);
  btn_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  enableOkButton(false);

  // Layout
  const auto form = new QFormLayout();
  form->addRow("SSID", ssid_);
  form->addRow("PSK", psk_);
  form->addRow("Priority", priority_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(form);
  rows->addWidget(btn_box_);

  setLayout(rows);

  // Connection
  connect(ssid_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(psk_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(btn_box_, &QDialogButtonBox::accepted, this, &self::accept);
  connect(btn_box_, &QDialogButtonBox::rejected, this, &self::reject);
}

QString AddWifiDialog::getSsid() const
{
  return ssid_->text();
}

QString AddWifiDialog::getPsk() const
{
  return psk_->text();
}

int AddWifiDialog::getPriority() const
{
  return priority_->value();
}

bool AddWifiDialog::isAcceptable() const
{
  const auto ssid = getSsid();
  if (ssid.isEmpty()) {
    return false;
  }

  const auto psk = getPsk();
  if (psk.length() < kWpaPskMinLength) {
    return false;
  }

  return true;
}

void AddWifiDialog::enableOkButton(bool enable)
{
  btn_box_->button(QDialogButtonBox::Ok)->setEnabled(enable);
}

void AddWifiDialog::onTextChanged()
{
  enableOkButton(isAcceptable());
}
}  // namespace bm
}  // namespace gui
