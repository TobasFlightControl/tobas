// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <QLineEdit>
#include <QWidget>

namespace tobas::qt
{
class IPv6Edit : public QWidget
{
  Q_OBJECT

  using super = QWidget;

  static constexpr size_t kNumFields = 8;

public:
  explicit IPv6Edit(QWidget* _parent = nullptr);

  void clear();

  bool isFilled() const;

  __uint128_t toInt() const;

  /** xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx */
  QString toString() const;

  void setFromInt(__uint128_t _address);

protected:
  bool eventFilter(QObject* _watched, QEvent* _event) override;

private:
  std::array<QLineEdit*, kNumFields> fields_;

  uint16_t getFieldValue(size_t _idx) const;
  void setFieldValue(size_t _idx, uint16_t _value);
};
}  // namespace tobas::qt
