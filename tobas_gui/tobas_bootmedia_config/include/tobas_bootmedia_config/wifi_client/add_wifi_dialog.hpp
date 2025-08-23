#pragma once

#include <map>

#include <QDialog>
#include <QDialogButtonBox>
#include <QLineEdit>

#include <tobas_qt_tools/widgets/password_edit.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas
{
namespace gui
{
namespace bm
{
class AddWifiDialog : public QDialog
{
  Q_OBJECT

  using self = AddWifiDialog;
  using super = QDialog;

public:
  explicit AddWifiDialog(QWidget* parent);

  QString getSsid() const;
  QString getPsk() const;
  int getPriority() const;

private:
  QLineEdit* ssid_;
  qt::PasswordEdit* psk_;
  qt::SpinBox* priority_;

  QDialogButtonBox* btn_box_;

  bool isAcceptable() const;
  void enableOkButton(bool enable);

private Q_SLOTS:
  void onTextChanged();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
