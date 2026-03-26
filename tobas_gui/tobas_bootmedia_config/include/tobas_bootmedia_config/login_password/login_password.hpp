#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/widgets/password_edit.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class LoginPasswordWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = LoginPasswordWidget;
  using super = BaseConfigWidget;

public:
  explicit LoginPasswordWidget();

  const char* name() const override;
  const char* title() const override;

  void reset() override;

private:
  tobas::qt::PasswordEdit* pswd1_;
  tobas::qt::PasswordEdit* pswd2_;

  tobas::qt::Label* warn_text_;

  QPushButton* write_button_;

private Q_SLOTS:
  void onTextChanged();
  void onWriteButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
