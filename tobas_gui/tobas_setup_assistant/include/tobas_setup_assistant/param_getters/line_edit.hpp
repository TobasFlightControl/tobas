#pragma once

#include <QLineEdit>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_LineEdit : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void textChanged(const QString& text);

public:
  explicit ParamGetterWidget_LineEdit(
    const QString& param_name,
    const QString& description_text = "",
    const QString& _default = "");

  QString get() const override;
  bool set(const QString& src) override;

private Q_SLOTS:
  void onTextChanged(const QString& text);

private:
  QLineEdit* line_;
};
}  // namespace setup_assistant
}  // namespace gui
