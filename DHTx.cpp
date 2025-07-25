
#include "DHTx.h"

os_timer_t DHTx::_timer;
uint8_t DHTx::_pin;
uint32_t DHTx::_maxcycles;
uint8_t DHTx::_state;
float DHTx::_temperature;
float DHTx::_humidity;
unsigned long DHTx::_lastRead;

DHTx::DHTx(uint8_t pin) {
  _pin = pin;
  _maxcycles = microsecondsToClockCycles(100);
  _state;
  _temperature = NAN;
  _humidity = NAN;
  _lastRead = 0;
}

void DHTx::begin() {
  pinMode(_pin, INPUT_PULLUP);
  os_timer_disarm(&_timer);
  os_timer_setfn(&_timer, (os_timer_func_t *) &_timerCallback, NULL);
  _state = 1;
  _nextMillis(100);
}

void DHTx::_nextMillis(unsigned long duration) {
  os_timer_arm(&_timer, duration, false);
}

uint32_t DHTx::_expectPulse(bool level) {
  uint32_t count = 0;
  while (digitalRead(_pin) == level) {
    if (count++ >= _maxcycles) {
      return 0; // Exceeded timeout, fail.
    }
  }
  return count;
}

void DHTx::_timerCallback(void *pArg) {
  os_timer_disarm(&_timer);

  switch (_state) {
    case 1:
      // Send start signal.
      // Go into high impedence state to let pull-up raise data line level and
      // start the reading process.
      digitalWrite(_pin, HIGH);
      _state = 2;
      _nextMillis(250);
      break;
    case 2:
      // First set data line low for 20 milliseconds.
      pinMode(_pin, OUTPUT);
      digitalWrite(_pin, LOW);
      _state = 3;
      _nextMillis(20);
      break;
    case 3:
      uint32_t _cycles[80];
      uint8_t dht_data[5];
      dht_data[0] = dht_data[1] = dht_data[2] = dht_data[3] = dht_data[4] = 0;
      {
        // Turn off interrupts temporarily because the next sections are timing critical
        // and we don't want any interruptions.
        InterruptLock lock;

        // End the start signal by setting data line high for 40 microseconds.
        digitalWrite(_pin, HIGH);
        delayMicroseconds(40);

        // Now start reading the data line to get the value from the DHT sensor.
        pinMode(_pin, INPUT_PULLUP);
        delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

        // First expect a low signal for ~80 microseconds followed by a high signal
        // for ~80 microseconds again.
        if (_expectPulse(LOW) == 0) {
          _state = 100;
          //          udpLog.print("DHT read error while expectPulse low");
          _nextMillis(10);
          return;
        }
        if (_expectPulse(HIGH) == 0) {
          _state = 100;
          //          udpLog.print("DHT read error while expectPulse high");
          _nextMillis(10);
          return;
        }

        // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
        // microsecond low pulse followed by a variable length high pulse.  If the
        // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
        // then it's a 1.  We measure the cycle count of the initial 50us low pulse
        // and use that to compare to the cycle count of the high pulse to determine
        // if the bit is a 0 (high state cycle count < low state cycle count), or a
        // 1 (high state cycle count > low state cycle count). Note that for speed all
        // the pulses are read into a array and then examined in a later step.
        for (int i = 0; i < 80; i += 2) {
          _cycles[i] = _expectPulse(LOW);
          _cycles[i + 1] = _expectPulse(HIGH);
        }
      } // Timing critical code is now complete.

      // Inspect pulses and determine which ones are 0 (high state cycle count < low
      // state cycle count), or 1 (high state cycle count > low state cycle count).
      for (int i = 0; i < 40; ++i) {
        uint32_t lowCycles  = _cycles[2 * i];
        uint32_t highCycles = _cycles[2 * i + 1];
        if ((lowCycles == 0) || (highCycles == 0)) {
          _state = 100;
          //          udpLog.print("DHT read error while inspect pulses");
          _nextMillis(10);
          return;
        }
        dht_data[i / 8] <<= 1;
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (highCycles > lowCycles) {
          // High cycles are greater than 50us low cycle count, must be a 1.
          dht_data[i / 8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
      }

      // Check we read 40 bits and that the checksum matches.
      if (dht_data[4] == ((dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]) & 0xFF)) {
        _state = 4;
        _humidity = dht_data[0];
        _temperature = dht_data[2];
        _lastRead = millis();
        _nextMillis(READ_PERIODIC);
      } else {
        _state = 100;
        //        udpLog.print("DHT checksum error");
        _nextMillis(10);
      }
      break;
    case 4:
      _state = 1;
      _nextMillis(100);
      break;
    case 100:
      _humidity = NAN;
      _temperature = NAN;
      pinMode(_pin, INPUT_PULLUP);
      _state = 1;
      _nextMillis(READ_ERROR_RETRY);
      break;
  }
}

float DHTx::readTemperature() {
  return _temperature;
}

float DHTx::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float DHTx::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float DHTx::readHumidity() {
  return _humidity;
}

float DHTx::computeHeatIndex(float temperature, float percentHumidity) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
         2.04901523 * temperature +
         10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return convertFtoC(hi);
}

