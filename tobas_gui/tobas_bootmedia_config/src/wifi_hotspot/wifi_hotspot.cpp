#include "tobas_bootmedia_config/wifi_hotspot/wifi_hotspot.hpp"

#include <QDebug>
#include <QFormLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace gui
{
namespace bm
{
WifiHotspotWidget::WifiHotspotWidget()
{
  ssid_ = new QLineEdit();
  psk_ = new qt::PasswordEdit();

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  // Layout
  const auto form = new QFormLayout();
  form->setHorizontalSpacing(30);
  form->addRow("New SSID", ssid_);
  form->addRow("New PSK", psk_);

  rows_->addLayout(form);
  rows_->addSpacing(30);
  qt::addWidgetCenter(write_button_, rows_);
  rows_->addStretch();

  // Connection
  connect(ssid_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(psk_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
}

const char* WifiHotspotWidget::name() const
{
  return "Wi-Fi Hotspot";
}

const char* WifiHotspotWidget::title() const
{
  return "Configure Wi-Fi Hotspot";
}

void WifiHotspotWidget::reset()
{
  ssid_->clear();
  psk_->clear();

  write_button_->setEnabled(false);
}

QString WifiHotspotWidget::getSsid() const
{
  return ssid_->text();
}

QString WifiHotspotWidget::getPsk() const
{
  return psk_->text();
}

bool WifiHotspotWidget::isAcceptable() const
{
  return !getSsid().isEmpty() && !getPsk().isEmpty();
}

void WifiHotspotWidget::onWriteButtonClicked()
{
  // TODO
}

void WifiHotspotWidget::onTextChanged()
{
  write_button_->setEnabled(isAcceptable());
}
}  // namespace bm
}  // namespace gui
