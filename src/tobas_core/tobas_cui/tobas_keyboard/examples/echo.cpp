// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iomanip>
#include <iostream>
#include <thread>

#include <tobas_keyboard/keyboard_reader.hpp>

#define HEX_STREAM(c) "0x" << setw(2) << setfill('0') << hex << uppercase << (int)c

using namespace std;

int main()
{
  tobas::keyboard::KeyboardReader keyboard;

  while (true) {
    const auto c = keyboard.readKey();
    if (c < 0) {
      cout << "Failed to read keyboard.\n";
    }
    else if (c == 0) {
    }
    else if (c < 0x21) {
      cout << "Special Command : " << HEX_STREAM(c) << '\n';
    }
    else {
      cout << "Normal Character: " << c << '\n';
    }

    // Show each key immediately, including when stdout is redirected.
    if (c != 0) {
      cout << flush;
    }

    this_thread::sleep_for(10ms);
  }
}
