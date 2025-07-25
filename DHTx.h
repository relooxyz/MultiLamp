#ifndef DHTx_h
#define DHTx_h

#include <Arduino.h>
#include <user_interface.h>
#include <DHT.h>

#define READ_PERIODIC 5000
#define READ_ERROR_RETRY 1000

class DHTx {
  public:
    DHTx(uint8_t pin);
    static void begin(void);
    static float readTemperature(void);
    static float readHumidity(void);
    static float convertCtoF(float);
    static float convertFtoC(float);
    static float computeHeatIndex(float temperature, float percentHumidity);

  private:
    static void _timerCallback(void *pArg);
    static void _nextMillis(unsigned long duration);
    static uint32_t _expectPulse(bool level);

    static os_timer_t _timer;
    static uint8_t _pin;
    static uint32_t _maxcycles;
    static uint8_t _state;
    static float _temperature;
    static float _humidity;
    static unsigned long _lastRead;
};

#endif
