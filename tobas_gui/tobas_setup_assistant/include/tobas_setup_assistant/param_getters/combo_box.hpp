#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_ComboBox : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_ComboBox;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void indexChanged(int index);
  void textChanged(const QString& text);

public:
  explicit ParamGetterWidget_ComboBox(const QString& param_name, const QString& description_text);

  QString getValue() const override;
  bool setValue(const QString& src) override;

  int currentIndex() const;
  void addChoice(const QString& item);
  void addChoices(const QStringList& items);
  void setChoices(const QStringList& items);

private Q_SLOTS:
  void onIndexChanged(int index);
  void onTextChanged(const QString& text);

private:
  qt::ComboBox* box_;
};
}  // namespace setup_assistant
}  // namespace gui
