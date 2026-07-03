// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_keyboard/utils.hpp"

#include <X11/XKBlib.h>  // Including this in the header may cause #define conflicts.

namespace tobas
{
namespace keyboard
{
XkbControlsPtr getKeyboardControls()
{
  // Open display
  const auto display = XOpenDisplay(nullptr);
  if (!display) {
    return nullptr;
  }

  // Get keyboard map
  const auto kb = XkbGetMap(display, XkbAllComponentsMask, XkbUseCoreKbd);
  if (!kb) {
    return nullptr;
  }

  // Get keyboard controls
  if (XkbGetControls(display, XkbAllControlsMask, kb) != Success) {
    return nullptr;
  }

  return kb->ctrls;
}

std::expected<uint16_t, const char*> getKeyboardRepeatInterval()
{
  const auto keyboard = getKeyboardControls();
  if (!keyboard) {
    return std::unexpected("Failed to get keyboard control.");
  }

  return keyboard->repeat_interval;
}
}  // namespace keyboard
}  // namespace tobas
