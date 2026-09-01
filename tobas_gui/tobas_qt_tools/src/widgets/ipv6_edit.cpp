// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/ipv6_edit.hpp"

#include <QDebug>
#include <QEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QRegularExpression>
#include <QRegularExpressionValidator>

#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace qt
{
namespace
{
constexpr char kNormalizeIpv6HextetOnFocusOutProperty[] = "normalizeIpv6HextetOnFocusOut";
}  // namespace

IPv6Edit::IPv6Edit(QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const QRegularExpression hex4(R"(^[0-9A-Fa-f]{0,4}$)");
  for (int i = kNumFields - 1; i >= 0; --i) {
    const auto field = new QLineEdit();
    field->setValidator(new QRegularExpressionValidator(hex4));

    field->setProperty(kNormalizeIpv6HextetOnFocusOutProperty, true);
    field->installEventFilter(this);

    cols->addWidget(field);
    if (i != 0) {
      cols->addWidget(new QLabel(":"));
    }

    fields_[i] = field;
  }

  clear();
}

void IPv6Edit::clear()
{
  for (auto& field : fields_) {
    field->setText("0000");
  }
}

bool IPv6Edit::isFilled() const
{
  for (auto& field : fields_) {
    if (field->text().isEmpty()) {
      return false;
    }
  }

  return true;
}

__uint128_t IPv6Edit::toInt() const
{
  __uint128_t res = 0;

  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 16 * i;
    const auto value = getFieldValue(i);
    res |= (static_cast<__uint128_t>(value) << shift);
  }

  return res;
}

QString IPv6Edit::toString() const
{
  QString res;

  for (int i = kNumFields - 1; i >= 0; --i) {
    const auto value = getFieldValue(i);
    res += QString::number(value, 16);
    if (i != 0) {
      res += ':';
    }
  }

  return res;
}

void IPv6Edit::setFromInt(__uint128_t address)
{
  for (size_t i = 0; i < kNumFields; ++i) {
    const auto shift = 16 * i;
    const auto value = static_cast<uint16_t>((address >> shift) & 0xFFFF);
    setFieldValue(i, value);
  }
}

bool IPv6Edit::eventFilter(QObject* watched, QEvent* event)
{
  if (event->type() == QEvent::FocusOut) {
    const auto field = qobject_cast<QLineEdit*>(watched);

    if (field && field->property(kNormalizeIpv6HextetOnFocusOutProperty).toBool()) {
      const auto text = field->text().trimmed();

      if (text.isEmpty()) {
        field->setText("0000");
        return super::eventFilter(watched, event);
      }

      bool ok = false;
      const auto value = text.toUInt(&ok, 16);

      if (ok && value <= 0xFFFF) {
        field->setText(QString("%1").arg(value, 4, 16, QChar('0')).toUpper());
      }
      else {
        qWarning() << "Failed to interpret as hex:" << text;
        field->setText("0000");
      }
    }
  }

  return QWidget::eventFilter(watched, event);
}

uint16_t IPv6Edit::getFieldValue(size_t idx) const
{
  const auto text = fields_.at(idx)->text();

  if (text.isEmpty()) {
    qWarning() << "Field" << idx << "is empty.";
    return 0;
  }

  bool ok = false;
  const auto value = text.toInt(&ok, 16);
  TOBAS_CHECK(ok);
  TOBAS_CHECK(0 <= value && value <= UINT16_MAX);
  return static_cast<uint16_t>(value);
}

void IPv6Edit::setFieldValue(size_t idx, uint16_t value)
{
  fields_.at(idx)->setText(QString::number(value, 16).rightJustified(4, '0'));
}
}  // namespace qt
}  // namespace tobas
