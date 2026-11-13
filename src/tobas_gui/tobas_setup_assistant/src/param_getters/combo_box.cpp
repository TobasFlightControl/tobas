// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/combo_box.hpp"

namespace tobas::gui::sa
{
ParamGetterWidget_ComboBox::ParamGetterWidget_ComboBox(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  box_ = new qt::ComboBox();
  rows_->addWidget(box_);

  connect(box_, qOverload<int>(&qt::ComboBox::currentIndexChanged), this, &self::onIndexChanged);
  connect(box_, &qt::ComboBox::currentTextChanged, this, &self::onTextChanged);
}

QString ParamGetterWidget_ComboBox::getValue() const
{
  return box_->currentText();
}

bool ParamGetterWidget_ComboBox::setValue(const QString& _src)
{
  if (!box_->contains(_src)) {
    return false;
  }

  box_->setCurrentText(_src);
  return true;
}

int ParamGetterWidget_ComboBox::currentIndex() const
{
  return box_->currentIndex();
}

void ParamGetterWidget_ComboBox::addChoice(const QString& _item)
{
  box_->addItem(_item);
}

void ParamGetterWidget_ComboBox::addChoices(const QStringList& _items)
{
  box_->addItems(_items);
}

void ParamGetterWidget_ComboBox::setChoices(const QStringList& _items)
{
  box_->clear();
  addChoices(_items);
}

void ParamGetterWidget_ComboBox::onIndexChanged(int _index)
{
  Q_EMIT indexChanged(_index);
}

void ParamGetterWidget_ComboBox::onTextChanged(const QString& _text)
{
  Q_EMIT textChanged(_text);
}
}  // namespace tobas::gui::sa
