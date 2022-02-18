// example i2c code for reading proximity data from VCNL36826S sensor

#include "mbed.h"
#include "string.h"
#include "VL6180X.h"

Serial pc(USBTX, USBRX, 921600);
DigitalOut led(LED1);

I2C i2c(PF_0, PF_1); //(PB_9, PB_8); // SDA, SCL
//SPI spi(PE_6, PE_5, PE_2); // MOSI, MISO, SCK
DigitalOut cs(PE_4);

// variables for encoder -> distance calculation
float counts_to_mm = 3.14159f*24.257f/4096.0f;
float revs_to_mm = 3.14159f*24.257f;
float starting_position = 0.0f;
int starting_counts = 0;
int num_revolutions = 0;
int new_enc_pos = 0;
int old_enc_pos = 0;

// initialize sensor
VL6180X tof;
int range;
float light;
int range_status;

// data variables
int prox = 0;
float position = 0;

Timer t;
float loop_time = 0.05;
Timer t2;
int samp1, samp2;
Timer t3;

// main loop
int main() {
    
    // start up
    pc.printf("Initializing.\n\r");
    
    i2c.frequency(400000); // set bus freq
    
//    spi.frequency(1000000); // set bus freq
//    spi.format(16,0); // try 12 bits? sensor is 12 bits, SPI mode 2 (inverted clock, sampled on falling edge)
    
    // perform any setup for the sensor here     
    tof.begin(&i2c);
    wait(1);
    
    t.reset();
    t.start();
    t2.reset();
    t2.start();
    t3.reset();
    t3.start();
//    cs = 0;
//    starting_counts = spi.write(0xFFFF);
//    cs = 1;
//    starting_counts = (starting_counts>>3);
//    starting_position = ((float)starting_counts)*counts_to_mm;  
    
    while (1) {

        t.reset();
                    
        // get data from TOF sensor and encoder
        t2.reset();
        range = tof.readRange();
        samp1 = t2.read_us();
//        t2.reset();
//        light = tof.readLux(VL6180X_ALS_GAIN_5);
//        samp2 = t2.read_us();
        
        
//        old_enc_pos = new_enc_pos;
//        cs = 0;
//        new_enc_pos = spi.write(0xFFFF);
//        cs = 1;
//        new_enc_pos = (new_enc_pos>>3);
//        if ((new_enc_pos>3000)&&(old_enc_pos<1000)){
//            num_revolutions -= 1;
//        } else if ((new_enc_pos<1000)&&(old_enc_pos>3000)){
//            num_revolutions += 1;
//        }
//        position = -(((float)new_enc_pos)*counts_to_mm + ((float)num_revolutions)*revs_to_mm - starting_position);
        
        // print data
        pc.printf("%f, %d\n\r", t3.read(), range);
//        pc.printf("%d, %f, %d, %d\n\r", range, light, samp1, samp2);
//        pc.printf("prox: %d, raw_enc: %d, pos: %.2f\n\r", prox, new_enc_pos, position); //new_enc_pos); //(enc_data>>3));
//        pc.printf("%d, %.2f\n\r", prox, position);
        
        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}