#ifndef BMP3_FUNCS_H
#define BMP3_FUNCS_H

#include "mbed.h"


// hardware prototypes
extern DigitalOut cs01;
extern DigitalOut cs11;
extern DigitalOut cs21;
extern DigitalOut cs31;
extern DigitalOut cs41;
extern DigitalOut cs51;
extern DigitalOut cs61;
extern DigitalOut cs71;
extern SPI spi1;

extern DigitalOut cs02;
extern DigitalOut cs12;
extern DigitalOut cs22;
extern DigitalOut cs32;
extern DigitalOut cs42;
extern DigitalOut cs52;
extern DigitalOut cs62;
extern DigitalOut cs72;
extern SPI spi2;


// function prototypes
void writeLow1(uint8_t pin);
void writeHigh1(uint8_t pin);

int8_t bmp_spi1_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bmp_spi1_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

void writeLow2(uint8_t pin);
void writeHigh2(uint8_t pin);

int8_t bmp_spi2_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bmp_spi2_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

void bmp_delay_ms(uint32_t msec);

#endif