// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sim
{
class BaseWorldWidget : public QWidget
{
  Q_OBJECT

public:
  virtual std::filesystem::path worldPath() const = 0;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
