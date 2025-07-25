
#include "MCP.h"

MCP::MCP(uint8_t pin_SDA, uint8_t pin_SCL) {
  _pin_SDA = pin_SDA;
  _pin_SCL = pin_SCL;
  _gpio_A = 0;
  _gpio_B = 0;
}

void MCP::begin() {
  Wire.begin(_pin_SDA, _pin_SCL);
  Wire.setClock(MCP23017_SPEED);
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(MCP23017_IODIRA); // IODIRA register
  Wire.write(0x00); // set all of port A to outputs
  Wire.endTransmission();
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(MCP23017_IODIRB); // IODIRB register
  Wire.write(0x00); // set all of port B to outputs
  Wire.endTransmission();
}

void MCP::reset() {
  _gpio_A = 0;
  _gpio_B = 0;
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(MCP23017_GPIOA);
  Wire.write(_gpio_A);
  Wire.endTransmission();
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(MCP23017_GPIOB);
  Wire.write(_gpio_B);
  Wire.endTransmission();
}

void MCP::digitalWrite(uint8_t pin, uint8_t data) {
  uint8_t gpioAB;
  uint8_t regValue;
  uint8_t bitNo;

  if (pin < 8) {
    gpioAB = MCP23017_GPIOA;
    bitNo = pin;
    bitWrite(_gpio_A, bitNo, data);
    regValue = _gpio_A;
  } else {
    gpioAB = MCP23017_GPIOB;
    bitNo = pin % 8;
    bitWrite(_gpio_B, bitNo, data);
    regValue = _gpio_B;
  }

  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(gpioAB);
  Wire.write(regValue);
  Wire.endTransmission();
}

