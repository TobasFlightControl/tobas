#include <stdexcept>

#include "../../include/tobas_keyboard_teleop/x11.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
XkbControlsPtr getKeyboardControls()
{
  Display* display;
  XkbDescPtr kb;

  // Open display
  if ((display = XOpenDisplay(NULL)) == NULL)
  {
    throw runtime_error("Couldn't open X11 display");
  }

  // Get keyboard map
  if ((kb = XkbGetMap(display, XkbAllComponentsMask, XkbUseCoreKbd)) == NULL)
  {
    throw runtime_error("Couldn't get keyboard map");
  }

  // Get keyboard controls
  if (XkbGetControls(display, XkbAllControlsMask, kb) != Success)
  {
    throw runtime_error("Couldn't get keyboard controls");
  }

  return kb->ctrls;
}
}  // namespace tobas_keyboard_teleop
