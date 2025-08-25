#include "tobas_bootmedia_config/login_password/login_password.hpp"

#include <filesystem>

#include <QDebug>
#include <QFormLayout>
#include <QVBoxLayout>

#include <tobas_crypt/crypt.hpp>
#include <tobas_crypt/yescrypt.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/string.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace bm
{
LoginPasswordWidget::LoginPasswordWidget()
{
  pswd1_ = new qt::PasswordEdit();
  pswd2_ = new qt::PasswordEdit();

  warn_text_ = new qt::Label();
  warn_text_->setTextColor(Qt::red);

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  // Layout
  const auto form = new QFormLayout();
  form->setHorizontalSpacing(kFormSpacing);
  form->addRow("New Password", pswd1_);
  form->addRow("Confirm Password", pswd2_);

  rows_->addLayout(form);
  rows_->addWidget(warn_text_);
  rows_->addSpacing(30);
  qt::addWidgetCenter(write_button_, rows_);
  rows_->addStretch();

  // Connection
  connect(pswd1_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(pswd2_, &QLineEdit::textChanged, this, &self::onTextChanged);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
}

const char* LoginPasswordWidget::name() const
{
  return "Login Password";
}

const char* LoginPasswordWidget::title() const
{
  return "Set Login Password";
}

void LoginPasswordWidget::reset()
{
  pswd1_->reset();
  pswd2_->reset();

  warn_text_->clear();

  write_button_->setEnabled(false);
}

void LoginPasswordWidget::onWriteButtonClicked()
{
  const auto shadow_path = fs::path(kRootPath) / "etc/shadow";
  const auto pswd = pswd1_->text().toStdString();

  if (!crypt::setShadowPassword(shadow_path, kUserName, pswd, crypt::Yescrypt())) {
    qt::qErrorBox(this, "Failed to update login password.");
    return;
  }

  qt::qInfoBox(this, "Login password was updated successfully.");
}

void LoginPasswordWidget::onTextChanged()
{
  const auto pswd1 = pswd1_->text();
  const auto pswd2 = pswd2_->text();

  // 空はダメ
  if (pswd1.isEmpty()) {
    warn_text_->setText("Please enter a password.");
    write_button_->setEnabled(false);
    return;
  }

  // 制御文字はダメ
  if (qt::containsControlChars(pswd1)) {
    warn_text_->setText("Password must not contain control characters.");
    write_button_->setEnabled(false);
    return;
  }

  // バイト長チェック
  constexpr int kMinPswdBytes = 8;
  constexpr int kMaxPswdBytes = 128;
  const int bytes = pswd1.toUtf8().size();
  if (bytes < kMinPswdBytes) {
    warn_text_->setText("Use at least " + QString::number(kMinPswdBytes) + " characters.");
    write_button_->setEnabled(false);
    return;
  }
  if (bytes > kMaxPswdBytes) {
    warn_text_->setText("Password is too long (max " + QString::number(kMaxPswdBytes) + " bytes).");
    write_button_->setEnabled(false);
    return;
  }

  // 複数の文字種を用いる
  bool has_lower = false, has_upper = false, has_digit = false, has_symbol = false;
  for (const auto& ch : pswd1) {
    if (ch.isLower()) {
      has_lower = true;
    }
    else if (ch.isUpper()) {
      has_upper = true;
    }
    else if (ch.isDigit()) {
      has_digit = true;
    }
    else {
      has_symbol = true;
    }
  }
  const auto classes = (has_lower ? 1 : 0) + (has_upper ? 1 : 0) + (has_digit ? 1 : 0) + (has_symbol ? 1 : 0);
  constexpr int kMinClasses = 1;  // TODO: 3種混合くらいを強制すべき？
  if (classes < kMinClasses) {
    warn_text_->setText(
      "Use a mix of letters, numbers, and symbols (at least " + QString::number(kMinClasses) + " types).");
    write_button_->setEnabled(false);
    return;
  }

  // 2つのパスワードが一致していること
  if (pswd1 != pswd2) {
    warn_text_->setText("Passwords do not match.");
    write_button_->setEnabled(false);
    return;
  }

  warn_text_->clear();
  write_button_->setEnabled(true);
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
