#include "tobas_bootmedia_config/wifi_hotspot/wifi_hotspot.hpp"

#include <QDebug>
#include <QFormLayout>
#include <QVBoxLayout>
#include <inja/inja.hpp>

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/util.hpp"

namespace fs = std::filesystem;

namespace tobas
{
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
  form->setHorizontalSpacing(kFormSpacing);
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
  psk_->reset();

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

void WifiHotspotWidget::onWriteButtonClicked()
{
  // Create data
  inja::json tpl_data;
  tpl_data["ssid"] = getSsid().toStdString();
  tpl_data["psk"] = getPsk().toStdString();

  // Get paths
  const auto tpl_path = getPkgShareDir() / "templates/hostapd.conf";
  const auto out_path = fs::path(kRootPath) / "etc/hostapd/hostapd.conf";

  // Generate file
  inja::Environment env;
  try {
    const auto temp = env.parse_template(tpl_path);
    env.write(temp, tpl_data, out_path);
  }
  catch (const std::exception& e) {
    qt::qErrorBox(this, "Failed to write AP configuration: " + QString(e.what()));
    return;
  }

  qt::qInfoBox(this, "Access point configuration was written successfully.");
}

void WifiHotspotWidget::onTextChanged()
{
  write_button_->setEnabled(isAcceptable());
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
