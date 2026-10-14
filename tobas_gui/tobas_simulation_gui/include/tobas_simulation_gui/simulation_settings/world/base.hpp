// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <QWidget>

namespace tobas::gui::sim
{
class BaseWorldWidget : public QWidget
{
  Q_OBJECT

public:
  virtual std::filesystem::path worldPath() const = 0;
};
}  // namespace tobas::gui::sim
