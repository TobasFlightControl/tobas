#include "tobas_setup_assistant/param_getters/combo_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_ComboBox::ParamGetterWidget_ComboBox(
  const QString& param_name,
  const QString& description_text,
  const QStringList& choices,
  const QString& _default)
  : super(param_name, description_text)
{
  box_ = new qt::ComboBox();
  rows_->addWidget(box_);

  box_->addItems(choices);

  if (!_default.isEmpty())
    box_->setCurrentText(_default);

  connect(box_, SIGNAL(qt::ComboBox::currentIndexChanged(int)), this, SLOT(onIndexChanged(int)));
  connect(box_, SIGNAL(qt::ComboBox::currentTextChanged(const QString&)), this, SLOT(onTextChanged(const QString&)));
}

QString ParamGetterWidget_ComboBox::get() const
{
  return box_->currentText();
}

bool ParamGetterWidget_ComboBox::set(const QString& src)
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
