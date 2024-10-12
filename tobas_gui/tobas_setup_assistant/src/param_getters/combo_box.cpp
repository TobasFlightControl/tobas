#include "tobas_setup_assistant/param_getters/combo_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_ComboBox::ParamGetterWidget_ComboBox(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  box_ = new qt::ComboBox();
  rows_->addWidget(box_);

  connect(box_, QOverload<int>::of(&qt::ComboBox::currentIndexChanged), this, &self::onIndexChanged);
  connect(box_, &qt::ComboBox::currentTextChanged, this, &self::onTextChanged);
}

QString ParamGetterWidget_ComboBox::getValue() const
{
  return box_->currentText();
}

bool ParamGetterWidget_ComboBox::setValue(const QString& src)
{
  if (!box_->contains(src))
    return false;

  box_->setCurrentText(src);
  return true;
}

int ParamGetterWidget_ComboBox::currentIndex() const
{
  return box_->currentIndex();
}

void ParamGetterWidget_ComboBox::addChoice(const QString& item)
{
  box_->addItem(item);
}

void ParamGetterWidget_ComboBox::addChoices(const QStringList& items)
{
  box_->addItems(items);
}

void ParamGetterWidget_ComboBox::setChoices(const QStringList& items)
{
  box_->clear();
  addChoices(items);
}

void ParamGetterWidget_ComboBox::onIndexChanged(int index)
{
  Q_EMIT indexChanged(index);
}

void ParamGetterWidget_ComboBox::onTextChanged(const QString& text)
{
  Q_EMIT textChanged(text);
}
}  // namespace setup_assistant
}  // namespace gui
