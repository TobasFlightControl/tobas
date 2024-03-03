#include <stdexcept>
#include <X11/XKBlib.h>  // ヘッダでインクルードすると#defineが衝突する恐れあり

#include "../include/tobas_std_tools/x11.hpp"

using namespace std;

namespace tobas_std
{
XkbControlsPtr getKeyboardControls()
{
  // Open display
  const auto display = XOpenDisplay(nullptr);
  if (display == nullptr)
    return nullptr;

  // Get keyboard map
  const auto kb = XkbGetMap(display, XkbAllComponentsMask, XkbUseCoreKbd);
  if (kb == nullptr)
    return nullptr;

  // Get keyboard controls
  if (XkbGetControls(display, XkbAllControlsMask, kb) != Success)
    return nullptr;

  return kb->ctrls;
}

uint16_t getKeyboardRepeatInterval()
{
  const auto keyboard = getKeyboardControls();
  if (keyboard == nullptr)
    throw runtime_error("Failed to get keyboard control.");

  return keyboard->repeat_interval;
}
}  // namespace tobas_std
