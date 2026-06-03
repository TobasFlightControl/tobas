// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include "./network.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class BaseNetworkWidget : public QWidget
{
  Q_OBJECT

public:
  virtual QString name() const = 0;

  virtual void reset() = 0;

  virtual bool load(const Network& src) = 0;
  virtual Network dump() const = 0;
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
