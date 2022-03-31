#include "bmp3_funcs.h"

// write cs pin on channel 1
void writeLow1(uint8_t pin){
    if (pin == 1){
        cs01.write(0);
    }
    else if (pin == 2){
        cs11.write(0);
    }
    else if (pin == 3){
        cs21.write(0);
    }
    else if (pin == 4){
        cs31.write(0);
    }
    else if (pin == 5){
        cs41.write(0);
    }
    else if (pin == 6){
        cs51.write(0);
    }
    else if (pin == 7){
        cs61.write(0);
    }
    else if (pin == 8){
        cs71.write(0);
    }
}

void writeHigh1(uint8_t pin){
    if (pin == 1){
        cs01.write(1);
    }
    else if (pin == 2){
        cs11.write(1);
    }
    else if (pin == 3){
        cs21.write(1);
    }
    else if (pin == 4){
        cs31.write(1);
    }
    else if (pin == 5){
        cs41.write(1);
    }
    else if (pin == 6){
        cs51.write(1);
    }
    else if (pin == 7){
        cs61.write(1);
    }
    else if (pin == 8){
        cs71.write(1);
    }
}

// write cs pin on channel 2
void writeLow2(uint8_t pin){
    if (pin == 1){
        cs02.write(0);
    }
    else if (pin == 2){
        cs12.write(0);
    }
    else if (pin == 3){
        cs22.write(0);
    }
    else if (pin == 4){
        cs32.write(0);
    }
    else if (pin == 5){
        cs42.write(0);
    }
    else if (pin == 6){
        cs52.write(0);
    }
    else if (pin == 7){
        cs62.write(0);
    }
    else if (pin == 8){
        cs72.write(0);
    }
}

void writeHigh2(uint8_t pin){
    if (pin == 1){
        cs02.write(1);
    }
    else if (pin == 2){
        cs12.write(1);
    }
    else if (pin == 3){
        cs22.write(1);
    }
    else if (pin == 4){
        cs32.write(1);
    }
    else if (pin == 5){
        cs42.write(1);
    }
    else if (pin == 6){
        cs52.write(1);
    }
    else if (pin == 7){
        cs62.write(1);
    }
    else if (pin == 8){
        cs72.write(1);
    }
}


// General Read and Write functions for channel 1
// read function: |0x80 done in library, dummy byte taken care of in library
int8_t bmp_spi1_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow1(cspin);
    spi1.write(reg_addr); // send read command to chip_id register (reg 0x00)
    for(int i = 0; i < len; i++){
        *(reg_data+i) = spi1.write(0x00); // read in 2nd byte = chip_id
    }
    writeHigh1(cspin);
    return 0;
}

int8_t bmp_spi1_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow1(cspin);
    spi1.write(reg_addr);
    if (len>1) {
        for(int i = 0; i < len-1; i++){
            spi1.write(*(reg_data+i)); // send alternating register address and register bytes in multi write
        }
    }
    else{
        spi1.write(reg_data[0]);
    }    
    writeHigh1(cspin);
    return 0;
}

// General Read and Write functions for channel 2
// read function: |0x80 done in library, dummy byte taken care of in library
int8_t bmp_spi2_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow2(cspin);
    spi2.write(reg_addr); // send read command to chip_id register (reg 0x00)
    for(int i = 0; i < len; i++){
        *(reg_data+i) = spi2.write(0x00); // read in 2nd byte = chip_id
    }
    writeHigh2(cspin);
    return 0;
}

int8_t bmp_spi2_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow2(cspin);
    spi2.write(reg_addr);
    if (len>1) {
        for(int i = 0; i < len-1; i++){
            spi2.write(*(reg_data+i)); // send alternating register address and register bytes in multi write
        }
    }
    else{
        spi2.write(reg_data[0]);
    }    
    writeHigh2(cspin);
    return 0;
}

// Delay function
void bmp_delay_ms(uint32_t msec){ //delay in milliseconds
    wait_ms(msec); 
}   
    
