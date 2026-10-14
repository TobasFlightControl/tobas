// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/ipv4_edit.hpp"

#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>

#include <tobas_qt_tools/validator/int_validator.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas::qt
{
namespace
{
constexpr char kEmptyMeansZeroProperty[] = "emptyMeansZero";
}  // namespace

IPv4Edit::IPv4Edit(QWidget* _parent) : super(_parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  for (int i = kNumFields - 1; i >= 0; --i) {
    const auto field = new QLineEdit();
    field->setValidator(new qt::IntValidator(0, UINT8_MAX));

    field->setProperty(kEmptyMeansZeroProperty, true);
    field->installEventFilter(this);

    cols->addWidget(field);
    if (i != 0) {
      cols->addWidget(new QLabel("."));
    }

    fields_[i] = field;
  }

  clear();
}

void IPv4Edit::clear()
{
  for (auto& field : fields_) {
    field->setText("0");
  }
}

bool IPv4Edit::isFilled() const
{
  for (auto& field : fields_) {
    if (field->text().isEmpty()) {
      return false;
    }
  }

  return true;
}

uint32_t IPv4Edit::toInt() const
{
  uint32_t res = 0;

  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 8 * i;
    const auto value = getFieldValue(i);
    res |= (static_cast<uint32_t>(value) << shift);
  }

  return res;
}

QString IPv4Edit::toString() const
{
  QString res;

  for (int i = kNumFields - 1; i >= 0; --i) {
    const auto value = getFieldValue(i);
    res += QString::number(value);
    if (i != 0) {
      res += '.';
    }
  }

  return res;
}

void IPv4Edit::setFromInt(uint32_t _address)
{
  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 8 * i;
    const auto value = (_address >> shift) & 0xFF;
    setFieldValue(i, value);
  }
}

bool IPv4Edit::eventFilter(QObject* _watched, QEvent* _event)
{
  if (_event->type() == QEvent::FocusOut) {
    const auto field = qobject_cast<QLineEdit*>(_watched);
    if (field && field->property(kEmptyMeansZeroProperty).toBool() && field->text().trimmed().isEmpty()) {
      field->setText("0");
    }
  }

  return super::eventFilter(_watched, _event);
}

uint8_t IPv4Edit::getFieldValue(size_t _idx) const
{
  const auto text = fields_.at(_idx)->text();

  if (text.isEmpty()) {
    return 0;
  }

  bool ok = false;
  const auto value = text.toInt(&ok, 10);
  TOBAS_CHECK(ok);
  TOBAS_CHECK(0 <= value && value <= UINT8_MAX);
  return static_cast<uint8_t>(value);
}

void IPv4Edit::setFieldValue(size_t _idx, uint8_t _value)
{
  fields_.at(_idx)->setText(QString::number(_value));
}
}  // namespace tobas::qt
