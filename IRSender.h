#ifndef IRSender_h
#define IRSender_h

#include "MCP.h"

class IRSender {
  public:
    IRSender(MCP *mcp, uint8_t pin);
    void sendACPanasonic(unsigned char *dataBuff, int nbits);
//    void packACPanasonic2(uint8_t *buff, uint8_t iPower, uint8_t iMode, uint8_t iTemp, uint8_t iSwing, uint8_t iFanSpeed);
    void sendACPanasonic2(unsigned char *dataBuff, int nbits);
    void sendACPanasonic2(uint8_t iPower, uint8_t iMode, uint8_t iTemp, uint8_t iSwing, uint8_t iFanSpeed);
    void sendACPanasonic2Toggle(uint8_t iToggleButton);
    void sendPack(int nbits, unsigned char *dataBuff, int dataBuff_len);
    void sendTestCancelTimer();
    void sendTestCancelTimer2();
    void sendTestOnOff();
    uint8_t reverse_byte(uint8_t x);

  private:
    MCP *_mcp;
    uint8_t _pin;
    int _halfPeriodicTime;
    void space(int time);
    void mark(int time);
};

#endif
