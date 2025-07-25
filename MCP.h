#ifndef MCP_h
#define MCP_h

#include <Arduino.h>
#include <Wire.h>

#define MCP23017_SPEED 1700000UL  // 1.7 MHz
#define MCP23017_ADDRESS 0x20
#define MCP23017_IODIRA 0x00
#define MCP23017_GPIOA 0x12
#define MCP23017_IODIRB 0x01
#define MCP23017_GPIOB 0x13

class MCP {
  public:
    MCP(uint8_t pin_SDA, uint8_t pin_SCL);
    void begin();
    void reset();
    void digitalWrite(uint8_t pin, uint8_t data);

  private:
    uint8_t _pin_SDA;
    uint8_t _pin_SCL;
    uint8_t _gpio_A;
    uint8_t _gpio_B;
};

#endif

