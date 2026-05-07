// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QMetaType>
#include <QString>

namespace tobas
{
namespace gui
{
namespace bm
{
class BootMedia
{
public:
  QString vendor;
  QString model;
  QString devnode;

  QString string() const;
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas

Q_DECLARE_METATYPE(tobas::gui::bm::BootMedia);
