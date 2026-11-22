// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <iomanip>
#include <ios>
#include <iostream>
#include <ostream>
#include <thread>

#include <tobas_keyboard/keyboard_reader.hpp>

using namespace std::chrono_literals;

int main()
{
  tobas::keyboard::KeyboardReader keyboard;

  while (true) {
    const auto c = keyboard.readKey();
    if (c < 0) {
      std::cout << "Failed to read keyboard.\n";
    }
    else if (c == 0) {
    }
    else if (c < 0x21) {
      std::cout << "Special Command : " << "0x" << std::setw(2) << std::setfill('0') << std::hex << std::uppercase
                << static_cast<int>(c) << '\n';
    }
    else {
      std::cout << "Normal Character: " << c << '\n';
    }

    // Show each key immediately, including when stdout is redirected.
    if (c != 0) {
      std::cout << std::flush;
    }

    std::this_thread::sleep_for(10ms);
  }
}
