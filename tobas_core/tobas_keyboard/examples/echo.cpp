#include <iomanip>
#include <iostream>
#include <thread>

#include <tobas_keyboard/keyboard_reader.hpp>

#define HEX_STREAM(c) "0x" << setw(2) << setfill('0') << hex << uppercase << (int)c

using namespace std;

int main()
{
  keyboard::KeyboardReader keyboard;

  while (true) {
    const auto c = keyboard.readKey();
    if (c < 0) {
      cout << "Failed to read keyboard." << endl;
    }
    else if (c == 0)
      ;
    else if (c < 0x21) {
      cout << "Special Command : " << HEX_STREAM(c) << endl;
    }
    else {
      cout << "Normal Character: " << c << endl;
    }

    this_thread::sleep_for(10ms);
  }
}
