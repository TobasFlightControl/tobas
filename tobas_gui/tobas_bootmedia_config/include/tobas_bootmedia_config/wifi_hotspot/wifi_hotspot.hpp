#pragma once

#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/password_edit.hpp>

#include "../base.hpp"

namespace gui
{
namespace bm
{
class WifiHotspotWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = WifiHotspotWidget;
  using super = BaseConfigWidget;

public:
  explicit WifiHotspotWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  QLineEdit* ssid_;
  qt::PasswordEdit* psk_;

  QPushButton* write_button_;

  QString getSsid() const;
  QString getPsk() const;

  bool isAcceptable() const;

private Q_SLOTS:
  void onTextChanged();
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
