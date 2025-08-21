#include "tobas_bootmedia_config/wifi_client/add_wifi_dialog.hpp"

#include <QFormLayout>
#include <QPushButton>
#include <QVBoxLayout>

namespace gui
{
namespace bm
{
AddWifiDialog::AddWifiDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add New Network");

  ssid_ = new QLineEdit();
  psk_ = new QLineEdit();

  btn_box_ = new QDialogButtonBox();
  btn_box_->setOrientation(Qt::Horizontal);
  btn_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  enableOkButton(false);

  // Layout
  const auto form = new QFormLayout();
  form->addRow("SSID", ssid_);
  form->addRow("PSK", psk_);

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

const QString AddWifiDialog::getSsid() const
{
  return ssid_->text();
}

const QString AddWifiDialog::getPsk() const
{
  return psk_->text();
}

bool AddWifiDialog::isAcceptable() const
{
  return !getSsid().isEmpty() && !getPsk().isEmpty();
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
