#ifndef Light_h
#define Light_h

#include "MCP.h"

class Light {
  public:
    Light(MCP *mcp, uint8_t pin);
    void setSignal(uint8_t signal_buff[], uint8_t signal_buff_len);
    void doReset();
    void doAction();

  private:
    MCP *_mcp;
    uint8_t _pin;
    boolean _last_state;
    uint8_t _signal_wait;
    uint8_t _signal_buff_len;
    uint8_t _signal_buff_idx;
    uint8_t _signal_buff[16];
};

#endif

