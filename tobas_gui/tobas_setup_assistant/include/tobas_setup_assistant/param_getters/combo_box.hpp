// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_ComboBox : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_ComboBox;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void indexChanged(int _index);
  void textChanged(const QString& _text);

public:
  explicit ParamGetterWidget_ComboBox(const QString& _param_name, const QString& _description_text = "");

  QString getValue() const override;
  bool setValue(const QString& _src) override;

  int currentIndex() const;
  void addChoice(const QString& _item);
  void addChoices(const QStringList& _items);
  void setChoices(const QStringList& _items);

private Q_SLOTS:
  void onIndexChanged(int _index);
  void onTextChanged(const QString& _text);

private:
  qt::ComboBox* box_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
