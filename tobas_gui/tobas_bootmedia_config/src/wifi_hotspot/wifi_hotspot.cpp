// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/wifi_hotspot/wifi_hotspot.hpp"

#include <QDebug>
#include <QFormLayout>
#include <QRegularExpression>
#include <inja/inja.hpp>

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/string.hpp>
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

  warn_text_ = new qt::Label();
  warn_text_->setTextColor(Qt::red);

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  // Layout
  const auto form = new QFormLayout();
  form->setHorizontalSpacing(kFormSpacing);
  form->addRow("New SSID", ssid_);
  form->addRow("New PSK", psk_);

  rows_->addLayout(form);
  rows_->addWidget(warn_text_);
  rows_->addSpacing(30);
  qt::addWidgetCenter(write_button_, rows_);
  rows_->addStretch();

  // Connection
  connect(ssid_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(psk_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
}

const char* WifiHotspotWidget::title() const
{
  return "Configure Wi-Fi Hotspot";
}

void WifiHotspotWidget::reset()
{
  ssid_->clear();
  psk_->reset();

  warn_text_->clear();

  write_button_->setEnabled(false);
}

bool WifiHotspotWidget::onConnected()
{
  return true;
}

QString WifiHotspotWidget::getSsid() const
{
  return ssid_->text();
}

QString WifiHotspotWidget::getPsk() const
{
  return psk_->text();
}

bool WifiHotspotWidget::checkSsid(QString& msg) const
{
  const auto ssid = getSsid();

  // Empty values are not allowed.
  if (ssid.isEmpty()) {
    msg = "Please enter your SSID.";
    return false;
  }

  // Control characters are not allowed.
  if (qt::containsControlChars(ssid)) {
    msg = "SSID must not contain control characters.";
    return false;
  }

  // Check the byte length.
  constexpr int kMaxSsidBytes = 128;
  const int bytes = ssid.toUtf8().size();
  if (bytes > kMaxSsidBytes) {
    msg = "SSID is too long (max " + QString::number(kMaxSsidBytes) + " bytes).";
    return false;
  }

  // Leading or trailing spaces work, but are not recommended.
  if (!ssid.isEmpty() && (ssid.front().isSpace() || ssid.back().isSpace())) {
    msg = "SSID should not have leading or trailing whitespace.";
    return false;
  }

  msg.clear();
  return true;
}

bool WifiHotspotWidget::checkPsk(QString& msg) const
{
  const auto psk = getPsk();

  // Empty values are not allowed.
  if (psk.isEmpty()) {
    msg = "Please enter your PSK.";
    return false;
  }

  // Use either ASCII or hex.
  if (!isValidAsciiPsk(psk) && !isValid64HexPsk(psk)) {
    msg = "PSK must be 8–63 ASCII printable characters, or 64 hex digits.";
    return false;
  }

  // Leading or trailing spaces work, but are not recommended.
  if (!psk.isEmpty() && (psk.front().isSpace() || psk.back().isSpace())) {
    msg = "PSK should not have leading or trailing whitespace.";
    return false;
  }

  msg.clear();
  return true;
}

bool WifiHotspotWidget::isValidAsciiPsk(const QString& psk)
{
  const QRegularExpression re(R"(^[ -~]{8,63}$)");
  return re.match(psk).hasMatch();
}

bool WifiHotspotWidget::isValid64HexPsk(const QString& psk)
{
  const QRegularExpression re(R"(^[0-9A-Fa-f]{64}$)");
  return re.match(psk).hasMatch();
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
  QString msg;

  if (!checkSsid(msg)) {
    warn_text_->setText(msg);
    write_button_->setEnabled(false);
    return;
  }

  if (!checkPsk(msg)) {
    warn_text_->setText(msg);
    write_button_->setEnabled(false);
    return;
  }

  warn_text_->clear();
  write_button_->setEnabled(true);
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
