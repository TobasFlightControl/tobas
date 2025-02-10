#pragma once

#include <QLineEdit>

#include "./base.hpp"

namespace gui
{
namespace sa
{
class ParamGetterWidget_LineEdit : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_LineEdit;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void textChanged(const QString& text);

public:
  explicit ParamGetterWidget_LineEdit(const QString& param_name, const QString& description_text);

  QString getValue() const override;
  bool setValue(const QString& src) override;

private Q_SLOTS:
  void onTextChanged(const QString& text);

private:
  QLineEdit* line_;
};
}  // namespace sa
}  // namespace gui
