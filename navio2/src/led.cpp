#include "../include/navio2/led.hpp"

// Output is inverted
#define OFF 1
#define ON 0

namespace navio
{
Led::Led()
{
}

bool Led::initialize()
{
  pin_R_ = new Pin(RPI_GPIO_4);
  pin_G_ = new Pin(RPI_GPIO_27);
  pin_B_ = new Pin(RPI_GPIO_6);

  if (pin_R_->init() && pin_G_->init() && pin_B_->init())
  {
    pin_R_->setMode(Pin::GpioModeOutput);
    pin_G_->setMode(Pin::GpioModeOutput);
    pin_B_->setMode(Pin::GpioModeOutput);

    // Switch of LED
    pin_R_->write(OFF);
    pin_G_->write(OFF);
    pin_B_->write(OFF);
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
      pin_R_->write(OFF);
      pin_G_->write(OFF);
      pin_B_->write(OFF);
      break;
    case Colors::Red:
      pin_R_->write(ON);
      pin_G_->write(OFF);
      pin_B_->write(OFF);
      break;
    case Colors::Green:
      pin_R_->write(OFF);
      pin_G_->write(ON);
      pin_B_->write(OFF);
      break;
    case Colors::Blue:
      pin_R_->write(OFF);
      pin_G_->write(OFF);
      pin_B_->write(ON);
      break;
    case Colors::Cyan:
      pin_R_->write(OFF);
      pin_G_->write(ON);
      pin_B_->write(ON);
      break;
    case Colors::Magenta:
      pin_R_->write(ON);
      pin_G_->write(OFF);
      pin_B_->write(ON);
      break;
    case Colors::Yellow:
      pin_R_->write(ON);
      pin_G_->write(ON);
      pin_B_->write(OFF);
      break;
    case Colors::White:
      pin_R_->write(ON);
      pin_G_->write(ON);
      pin_B_->write(ON);
      break;
  }
}
}  // namespace navio
