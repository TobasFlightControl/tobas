#pragma once

#include <map>

#include <QDialog>
#include <QDialogButtonBox>
#include <QLineEdit>

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

  const QString getSsid() const;
  const QString getPsk() const;

private:
  QLineEdit* ssid_;
  QLineEdit* psk_;

  QDialogButtonBox* btn_box_;

  bool isAcceptable() const;
  void enableOkButton(bool enable);

private Q_SLOTS:
  void onTextChanged();
};
}  // namespace bm
}  // namespace gui
