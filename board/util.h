// Define to prevent recursive inclusion
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

// Initialization Functions
void BLDC_Init(void);

// General Functions
void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepLong(uint8_t freq);
void beepShort(uint8_t freq);
void beepShortMany(uint8_t cnt, int8_t dir);
void calcAvgSpeed(void);

// Poweroff Functions
void poweroff(void);
void poweroffPressCheck(void);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);

uint8_t crc_checksum(uint8_t *dat, int len, const uint8_t poly);
void delay(uint32_t a);

#endif
