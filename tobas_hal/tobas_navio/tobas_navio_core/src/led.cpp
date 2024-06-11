#include "../include/tobas_navio_core/led.hpp"

namespace navio
{
Led::Led() : pin_R_(RPI_GPIO_4), pin_G_(RPI_GPIO_27), pin_B_(RPI_GPIO_6)
{
}

bool Led::initialize()
{
  if (pin_R_.init() && pin_G_.init() && pin_B_.init())
  {
    pin_R_.setMode(Pin::OUTPUT);
    pin_G_.setMode(Pin::OUTPUT);
    pin_B_.setMode(Pin::OUTPUT);

    // Switch of LED
    pin_R_.write(kOffValue);
    pin_G_.write(kOffValue);
    pin_B_.write(kOffValue);
  }
  else
  {
    return false;
  }

  return true;
}

void Led::setColor(Colors color)
{
  switch (color)
  {
    case Colors::Black:
      pin_R_.write(kOffValue);
      pin_G_.write(kOffValue);
      pin_B_.write(kOffValue);
      break;
    case Colors::Red:
      pin_R_.write(kOnValue);
      pin_G_.write(kOffValue);
      pin_B_.write(kOffValue);
      break;
    case Colors::Green:
      pin_R_.write(kOffValue);
      pin_G_.write(kOnValue);
      pin_B_.write(kOffValue);
      break;
    case Colors::Blue:
      pin_R_.write(kOffValue);
      pin_G_.write(kOffValue);
      pin_B_.write(kOnValue);
      break;
    case Colors::Cyan:
      pin_R_.write(kOffValue);
      pin_G_.write(kOnValue);
      pin_B_.write(kOnValue);
      break;
    case Colors::Magenta:
      pin_R_.write(kOnValue);
      pin_G_.write(kOffValue);
      pin_B_.write(kOnValue);
      break;
    case Colors::Yellow:
      pin_R_.write(kOnValue);
      pin_G_.write(kOnValue);
      pin_B_.write(kOffValue);
      break;
    case Colors::White:
      pin_R_.write(kOnValue);
      pin_G_.write(kOnValue);
      pin_B_.write(kOnValue);
      break;
  }
}
}  // namespace navio
