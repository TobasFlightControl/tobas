#include <stdexcept>
#include <X11/XKBlib.h>  // NOTE: ヘッダでインクルードすると#defineが衝突する恐れあり

#include "../include/tobas_keyboard/utils.hpp"

using namespace std;

namespace keyboard
{
XkbControlsPtr getKeyboardControls()
{
  // Open display
  const auto display = XOpenDisplay(nullptr);
  if (!display)
    return nullptr;

  // Get keyboard map
  const auto kb = XkbGetMap(display, XkbAllComponentsMask, XkbUseCoreKbd);
  if (!kb)
    return nullptr;

  // Get keyboard controls
  if (XkbGetControls(display, XkbAllControlsMask, kb) != Success)
    return nullptr;

  return kb->ctrls;
}

double getKeyboardRepeatInterval()
{
  const auto keyboard = getKeyboardControls();
  if (!keyboard)
    throw runtime_error("Failed to get keyboard control.");

  return keyboard->repeat_interval * 1e-3;
}
}  // namespace keyboard
