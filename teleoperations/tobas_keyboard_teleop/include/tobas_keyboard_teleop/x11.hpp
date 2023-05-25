#pragma once

#include <X11/Xlib.h>
#include <X11/XKBlib.h>

namespace tobas_keyboard_teleop
{
XkbControlsPtr getKeyboardControls();
}
