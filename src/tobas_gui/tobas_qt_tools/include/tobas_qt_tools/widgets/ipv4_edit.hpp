// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <QLineEdit>
#include <QWidget>

namespace tobas::qt
{
class IPv4Edit : public QWidget
{
  Q_OBJECT

  using super = QWidget;

  static constexpr size_t kNumFields = 4;

public:
  explicit IPv4Edit(QWidget* _parent = nullptr);

  void clear();

  bool isFilled() const;

  uint32_t toInt() const;

  /** xxx.xxx.xxx.xxx */
  QString toString() const;

  void setFromInt(uint32_t _address);

protected:
  bool eventFilter(QObject* _watched, QEvent* _event) override;

private:
  std::array<QLineEdit*, kNumFields> fields_;

  uint8_t getFieldValue(size_t _idx) const;
  void setFieldValue(size_t _idx, uint8_t _value);
};
}  // namespace tobas::qt
