
#include "IRSender.h"

// AC PANASONIC
#define ACPANASONIC_HDR_MARK  3500    // 3500
#define ACPANASONIC_HDR_SPACE 3500
#define ACPANASONIC_BIT_MARK  900     // 720 or 860
#define ACPANASONIC_ONE_SPACE  2600   // 2400 or 2600
#define ACPANASONIC_ZERO_SPACE 900    // 950 or 890
#define ACPANASONIC_SEPARATOR_MARK 900
#define ACPANASONIC_SEPARATOR_SPACE 13900

// AC PANASONIC 2
#define ACPANASONIC2_HDR_MARK  3500
#define ACPANASONIC2_HDR_SPACE 1900
#define ACPANASONIC2_BIT_MARK  350
#define ACPANASONIC2_ONE_SPACE  1450
#define ACPANASONIC2_ZERO_SPACE 550

static uint8_t preSignal[] = { 0x40, 0x04, 0x07, 0x20, 0x00, 0x00, 0x00, 0x60};
static uint8_t signalEion[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0x86, 0xCC, 0x58};
static uint8_t signalPatrol[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0xC6, 0xCC, 0x38};
static uint8_t signalPowerfullQuite[] = { 0x40, 0x04, 0x07, 0x20, 0x01, 0x39, 0x4C, 0x2A};

IRSender::IRSender(MCP *mcp, uint8_t pin) {
  _mcp = mcp;
  _pin = pin;
  // The khz value controls the modulation frequency in kilohertz.
  int khz = 38;
  // T = 1/f but we need T/2 in microsecond and f is in kHz
  //38 kHz -> T = 26.31 microsec (periodic time), half of it is 13
  _halfPeriodicTime = 500 / khz;

}

void IRSender::sendACPanasonic(unsigned char *dataBuff, int nbits) {
  int dataIdx = 0;
  int dataBit = 0;
  int dataBitSeparator = 0;
  unsigned int data;

  mark(ACPANASONIC_HDR_MARK);
  space(ACPANASONIC_HDR_SPACE);

  for (int i = 0; i < nbits; i++) {
    if (dataBit == 8) {
      dataBit = 0;
      dataIdx++;
    }
    if (dataBit == 0) {
      data = dataBuff[dataIdx];
    }
    if (dataBitSeparator == 32) {
      dataBitSeparator = 0;
    }
    mark(ACPANASONIC_BIT_MARK);
    if (data & 0x80) {
      space(ACPANASONIC_ONE_SPACE);
    }
    else {
      space(ACPANASONIC_ZERO_SPACE);
    }
    if (dataBitSeparator == 31) {
      mark(ACPANASONIC_HDR_MARK);
      space(ACPANASONIC_HDR_SPACE);
    }
    data <<= 1;
    dataBit++;
    dataBitSeparator++;
  }
  mark(ACPANASONIC_SEPARATOR_MARK);
  space(ACPANASONIC_SEPARATOR_SPACE);
}

void IRSender::sendPack(int nbits, unsigned char *dataBuff, int dataBuff_len) {
  int chunkSize = 0;
  int idxByte = 0;
  int bitNo = 0;
  while ((bitNo < nbits) && (idxByte < dataBuff_len)) {
    chunkSize = dataBuff[idxByte];
    bitNo += chunkSize;
    idxByte++;
    sendACPanasonic(&dataBuff[idxByte], chunkSize);
    idxByte += chunkSize / 8;
    if (chunkSize % 8 > 0) {
      idxByte++;
    }
  }
}

void IRSender::sendACPanasonic2(unsigned char *dataBuff, int nbits) {
  int dataIdx = 0;
  int dataBit = 0;
  unsigned int data;

  mark(ACPANASONIC2_HDR_MARK);
  space(ACPANASONIC2_HDR_SPACE);
  for (int i = 0; i < nbits; i++) {
    if (dataBit == 8) {
      dataBit = 0;
      dataIdx++;
    }
    if (dataBit == 0) {
      data = dataBuff[dataIdx];
    }
    mark(ACPANASONIC2_BIT_MARK);
    if (data & 0x80) {
      space(ACPANASONIC2_ONE_SPACE);
    }
    else {
      space(ACPANASONIC2_ZERO_SPACE);
    }
    data <<= 1;
    dataBit++;
  }
  mark(ACPANASONIC2_BIT_MARK);
  space(ACPANASONIC_SEPARATOR_SPACE);
}

//void IRSender::packACPanasonic2(uint8_t *buff, uint8_t iPower, uint8_t iMode, uint8_t iTemp, uint8_t iSwing, uint8_t iFanSpeed) {
//  uint8_t dataBuff[] = {
//    0x40, 0x04, 0x07, 0x20,
//    0x00, 0x00, 0x00, 0x01,
//    0x00, 0x00, 0x00, 0x70,
//    0x07, 0x00, 0x00, 0x81,
//    0x00, 0x00, 0x00
//  };
//
//  dataBuff[5] = reverse_byte(B00001000 | (iPower & B00000001) | ((iMode & B00001111) << 4));
//  dataBuff[6] = reverse_byte((iTemp & B00011111) << 1);
//  dataBuff[8] = reverse_byte((iSwing & B00001111) | ((iFanSpeed & B00001111) << 4));
//  uint16_t checksum = 0;
//  for (int i = 0; i < 18; i++) {
//    checksum += reverse_byte(dataBuff[i]);
//  }
//  dataBuff[18] = reverse_byte(checksum & 0xFF);
//
//  for (int i = 0; i < 19; i++) {
//    buff[i] = dataBuff[i];
//  }
//}

void IRSender::sendACPanasonic2(uint8_t iPower, uint8_t iMode, uint8_t iTemp, uint8_t iSwing, uint8_t iFanSpeed) {
  uint8_t dataBuff[] = {
    0x40, 0x04, 0x07, 0x20,
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x70,
    0x07, 0x00, 0x00, 0x81,
    0x00, 0x00, 0x00
  };

  dataBuff[5] = reverse_byte(B00001000 | (iPower & B00000001) | ((iMode & B00001111) << 4));
  dataBuff[6] = reverse_byte((iTemp & B00011111) << 1);
  dataBuff[8] = reverse_byte((iSwing & B00001111) | ((iFanSpeed & B00001111) << 4));
  uint16_t checksum = 0;
  for (int i = 0; i < 18; i++) {
    checksum += reverse_byte(dataBuff[i]);
  }
  dataBuff[18] = reverse_byte(checksum & 0xFF);

  sendACPanasonic2(preSignal, 64);
  sendACPanasonic2(dataBuff, 152);
}

void IRSender::sendACPanasonic2Toggle(uint8_t iToggleButton) {
  switch (iToggleButton) {
    case 1:
      sendACPanasonic2(preSignal, 64);
      sendACPanasonic2(signalEion, 64);
      break;
    case 2:
      sendACPanasonic2(preSignal, 64);
      sendACPanasonic2(signalPatrol, 64);
      break;
    case 3:
      sendACPanasonic2(preSignal, 64);
      sendACPanasonic2(signalPowerfullQuite, 64);
      break;
  }
}

void IRSender::mark(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  long beginning = micros();
  while (micros() - beginning < time) {
    _mcp->digitalWrite(_pin, HIGH);
    delayMicroseconds(_halfPeriodicTime);
    _mcp->digitalWrite(_pin, LOW);
    delayMicroseconds(_halfPeriodicTime);
  }
}

/* Leave pin off for time (given in microseconds) */
void IRSender::space(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  _mcp->digitalWrite(_pin, LOW);
  if (time > 0) delayMicroseconds(time);
}

void IRSender::sendTestCancelTimer() {
  // Cancel Timer Button
  unsigned char dataBuff[] = {
    0x20, 0xFE, 0xFE, 0x1C, 0x1C,
    0x20, 0xFD, 0xFD, 0x1C, 0x1C,
    0x20, 0xFE, 0xFE, 0xBC, 0xBC,
    0x20, 0xFD, 0xFD, 0xBC, 0xBC,
    0x20, 0x64, 0x64, 0x2C, 0x2C,
    0x20, 0xC9, 0xC9, 0x2C, 0x2C
  };
  sendPack(192, (unsigned char *) &dataBuff, sizeof(dataBuff));
}

void IRSender::sendTestCancelTimer2() {
  // Cancel Timer Button
  unsigned char dataBuff[] = { 0x40, 0x04, 0x07, 0x20, 0x00, 0x1C, 0x2C, 0x01, 0x8C, 0x00, 0x16, 0x90, 0x01, 0x00, 0x00, 0x01, 0x24, 0x00, 0x1D };
  sendACPanasonic2(preSignal, 64);
  sendACPanasonic2(dataBuff, 152);
}

void IRSender::sendTestOnOff() {
  // OFF/ON Button
  unsigned char dataBuff[] = {
    0x40, 0x34, 0x34, 0x40, 0x40, 0x34, 0x34, 0x40, 0x40,
    0x40, 0x0B, 0x0B, 0x6C, 0x6C, 0x0B, 0x0B, 0x6C, 0x6C
  };
  sendPack(128, (unsigned char *) &dataBuff, sizeof(dataBuff));
}

uint8_t IRSender::reverse_byte(uint8_t x) {
  static const uint8_t table[] = {
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
  };
  return table[x];
}
