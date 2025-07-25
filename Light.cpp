
#include "Light.h"

Light::Light(MCP *mcp, uint8_t pin) {
  _mcp = mcp;
  _pin = pin;
  _last_state = LOW;
  _signal_buff_len = 0;
}

void Light::setSignal(uint8_t signal_buff[], uint8_t signal_buff_len) {
  memcpy(_signal_buff, signal_buff, signal_buff_len);
  _signal_buff_len = signal_buff_len;
  _signal_buff_idx = _signal_buff_len - 1;
  _signal_wait = 0;
}

void Light::doReset() {
  _mcp->digitalWrite(_pin, LOW);
  _last_state = LOW;
  _signal_buff_idx = _signal_buff_len - 1;
  _signal_wait = 0;
}

void Light::doAction() {
  if (_signal_buff_len == 0) {
    return;
  }
  if (_signal_wait == 0) {
    _signal_buff_idx++;
    if (_signal_buff_idx == _signal_buff_len) {
      _signal_buff_idx = 0;
    }
    _signal_wait = _signal_buff[_signal_buff_idx];
    if (_signal_wait == 0) {
      // ignore this mark time
      _signal_buff_idx++;
      if (_signal_buff_idx == _signal_buff_len) {
        _signal_buff_idx = 0;
      }
      _signal_wait = _signal_buff[_signal_buff_idx];
    }
    if (_signal_wait > 0) {
      _signal_wait--;
    }
    if (_signal_buff_idx % 2 == 0) {
      if (_last_state == LOW) {
        _mcp->digitalWrite(_pin, HIGH);
        _last_state = HIGH;
      }
    } else {
      if (_last_state == HIGH) {
        _mcp->digitalWrite(_pin, LOW);
        _last_state = LOW;
      }
    }
  } else {
    _signal_wait--;
  }
}

