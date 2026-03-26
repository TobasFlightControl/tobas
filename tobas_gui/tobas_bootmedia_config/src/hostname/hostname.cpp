#include "tobas_bootmedia_config/hostname/hostname.hpp"

#include <QDebug>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <inja/inja.hpp>

#include <tobas_qt_tools/message.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/util.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace bm
{
HostnameWidget::HostnameWidget()
{
  read_button_ = new QPushButton("Read");
  write_button_ = new QPushButton("Write");

  read_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  hostname_ = new QLineEdit();
  hostname_->setMaxLength(HOST_NAME_MAX);

  warn_text_ = new tobas::qt::Label();
  warn_text_->setTextColor(Qt::red);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(write_button_);
  cols->addStretch();

  const auto form = new QFormLayout();
  form->setHorizontalSpacing(kFormSpacing);
  form->addRow("Hostname", hostname_);

  rows_->addLayout(cols);
  rows_->addLayout(form);
  rows_->addWidget(warn_text_);
  rows_->addStretch();

  // Connection
  connect(hostname_, &QLineEdit::textChanged, this, &self::onHostnameChanged);
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
}

const char* HostnameWidget::name() const
{
  return "Hostname";
}

const char* HostnameWidget::title() const
{
  return "Set Linux Hostname";
}

void HostnameWidget::reset()
{
  read_button_->setEnabled(true);
  write_button_->setEnabled(false);

  hostname_->clear();
  warn_text_->clear();
}

QString HostnameWidget::getHostname() const
{
  return hostname_->text();
}

bool HostnameWidget::writeHostnameFile(const QString& hostname)
{
  const auto hostname_file_path = hostnameFilePath();
  const auto hostname_file_content = (hostname + '\n').toStdString();

  if (!str::writeText(hostname_file_path, hostname_file_content)) {
    tobas::qt::qErrorBox(this, "Failed to write hostname to " + QString::fromStdString(hostname_file_path) + ".");
    return false;
  }

  return true;
}

bool HostnameWidget::writeHostsFile(const QString& hostname)
{
  // Create data
  inja::json tpl_data;
  tpl_data["hostname"] = hostname.toStdString();

  // Get paths
  const auto tpl_path = getPkgShareDir() / "templates/hosts";
  const auto out_path = fs::path(kRootPath) / "etc/hosts";

  // Generate file
  inja::Environment env;
  try {
    const auto temp = env.parse_template(tpl_path);
    env.write(temp, tpl_data, out_path);
  }
  catch (const std::exception& e) {
    tobas::qt::qErrorBox(this, "Failed to write hostname to " + QString::fromStdString(out_path) + ": " + e.what());
    return false;
  }

  return true;
}

std::string HostnameWidget::hostnameFilePath()
{
  return std::string(kRootPath) + "/etc/hostname";
}

std::string HostnameWidget::hostsFilePath()
{
  return std::string(kRootPath) + "/etc/hosts";
}

void HostnameWidget::onHostnameChanged(const QString& hostname)
{
  // 空はダメ
  if (hostname.isEmpty()) {
    warn_text_->setText("Please enter a hostname.");
    write_button_->setEnabled(false);
    return;
  }

  // FQDN禁止
  if (hostname.contains('.')) {
    warn_text_->setText("Dots are not allowed in the static hostname.");
    write_button_->setEnabled(false);
    return;
  }

  // ASCII前提でバイト長チェック
  const auto hostname_bytes = hostname.toUtf8().size();
  if (hostname_bytes > HOST_NAME_MAX) {
    warn_text_->setText("Hostname is too long (max " + QString::number(HOST_NAME_MAX) + " characters).");
    write_button_->setEnabled(false);
    return;
  }

  // 小文字の英字・数字・ハイフンのみ許可
  for (auto ch : hostname) {
    const auto u = ch.unicode();
    const auto is_lower = (u >= 'a' && u <= 'z');
    const auto is_digit = (u >= '0' && u <= '9');
    const auto is_hyphen = (u == '-');
    if (!(is_lower || is_digit || is_hyphen)) {
      warn_text_->setText("Use lowercase letters a–z, digits 0–9, and hyphens (-) only.");
      write_button_->setEnabled(false);
      return;
    }
  }

  // 先頭・末尾のハイフン禁止
  if (hostname.startsWith('-') || hostname.endsWith('-')) {
    warn_text_->setText("Hostname must not start or end with a hyphen (-).");
    write_button_->setEnabled(false);
    return;
  }

  warn_text_->clear();
  write_button_->setEnabled(true);
}

void HostnameWidget::onReadButtonClicked()
{
  std::string file_content;
  if (!str::readText(hostnameFilePath(), file_content)) {
    tobas::qt::qErrorBox(this, "Failed to read the current hostname.");
    return;
  }

  const auto hostname = QString::fromStdString(file_content).trimmed();  // 末尾の改行コードを削除
  hostname_->setText(hostname);

  tobas::qt::qInfoBox(this, "Hostname was read successfully.");
}

void HostnameWidget::onWriteButtonClicked()
{
  const auto hostname = getHostname();

  if (!writeHostnameFile(hostname)) {
    return;
  }
  if (!writeHostsFile(hostname)) {
    return;
  }

  tobas::qt::qInfoBox(this, "Hostname was written successfully.");
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
