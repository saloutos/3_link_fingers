// example i2c code for reading proximity data from VCNL36826S sensor

#include "mbed.h"
#include "string.h"
#include "VL6180X.h"

Serial pc(USBTX, USBRX, 460800);
DigitalOut led(LED1);

I2C i2c1(PF_0, PF_1); //(PB_9, PB_8); // SDA, SCL
I2C i2c2(PD_13, PD_12); 
// SPI spi(PE_6, PE_5, PE_2); // MOSI, MISO, SCK
// DigitalOut cs(PE_4);

// variables for encoder -> distance calculation
// float counts_to_mm = 3.14159f*24.257f/4096.0f;
// float revs_to_mm = 3.14159f*24.257f;
// float starting_position = 0.0f;
// int starting_counts = 0;
// int num_revolutions = 0;
// int new_enc_pos = 0;
// int old_enc_pos = 0;

// initialize sensors
VL6180X tof1;
VL6180X tof2;
VL6180X tof3;
VL6180X tof4;
VL6180X tof5;
VL6180X tof6;
int range[6];
int range_status[6];

// data variables
uint8_t MUX_ADDR = (0x70<<1);
// int prox[3];
// float position = 0;

Timer t;
float loop_time = 0.005; // 200hz is probably maximum sample rate since that is pressure sensor rate too
uint16_t range_period = 20;
Timer t2;
int samp1, samp2, samp3;
Timer t3;

void set_mux1(uint8_t channel){
    // write to mux address
    char buffer[1];
    buffer[0] = (1 << channel);
    int result = 1; 
    result = i2c1.write(MUX_ADDR, buffer, 1);
    // pc.printf("C: %d, r: %d\n\r", channel, result);
}

void set_mux2(uint8_t channel){
    // write to mux address
    char buffer[1];
    buffer[0] = (1 << channel);
    int result = 1; 
    result = i2c2.write(MUX_ADDR, buffer, 1);
    // pc.printf("C: %d, r: %d\n\r", channel, result);
}

// main loop
int main() {
    
    // start up
    pc.printf("Initializing.\n\r");
    
    i2c1.frequency(400000); // set bus freq
    i2c2.frequency(400000); 

    // spi.frequency(1000000); // set bus freq
    // spi.format(16,0); // try 12 bits? sensor is 12 bits, SPI mode 2 (inverted clock, sampled on falling edge)
    
    // perform any setup for the sensor here
    // TODO: call load setting function, start continuous ranging
    pc.printf("Sensor 1...\n\r");
    set_mux1(2); // channel 2 on mux 
    wait_us(100);
    if(!tof1.begin(&i2c1)){
        pc.printf("Sensor 1 init failed.\n\r");
    }
    wait_us(100);
    tof1.stopRangeContinuous();
    wait_us(100);
    tof1.startRangeContinuous(range_period);
    wait_us(1000);
    pc.printf("Sensor 2...\n\r");
    set_mux1(3); // channel 3 on mux
    wait_us(100);
    if(!tof2.begin(&i2c1)){
        pc.printf("Sensor 2 init failed.\n\r");
    }
    wait_us(100);
    tof2.stopRangeContinuous();
    wait_us(100);
    tof2.startRangeContinuous(range_period);
    wait_us(1000);
    pc.printf("Sensor 3...\n\r");
    set_mux1(4); // channel 4 on mux
    wait_us(100);
    if(!tof3.begin(&i2c1)){
        pc.printf("Sensor 3 init failed.\n\r");
    }
    wait_us(100);
    tof3.stopRangeContinuous();
    wait_us(100);
    tof3.startRangeContinuous(range_period);
    wait_us(1000);
    pc.printf("Sensor 4...\n\r");
    set_mux2(2); // channel 2 on mux 
    wait_us(100);
    if(!tof4.begin(&i2c2)){
        pc.printf("Sensor 4 init failed.\n\r");
    }
    wait_us(100);
    tof4.stopRangeContinuous();
    wait_us(100);
    tof4.startRangeContinuous(range_period);
    wait_us(1000);
    pc.printf("Sensor 5...\n\r");
    set_mux2(3); // channel 3 on mux
    wait_us(100);
    if(!tof5.begin(&i2c2)){
        pc.printf("Sensor 5 init failed.\n\r");
    }
    wait_us(100);
    tof5.stopRangeContinuous();
    wait_us(100);
    tof5.startRangeContinuous(range_period);
    wait_us(1000);
    pc.printf("Sensor 6...\n\r");
    set_mux2(4); // channel 4 on mux
    wait_us(100);
    if(!tof6.begin(&i2c2)){
        pc.printf("Sensor 6 init failed.\n\r");
    }
    wait_us(100);
    tof6.stopRangeContinuous();
    wait_us(100);
    tof6.startRangeContinuous(range_period);
    wait_us(1000);
    
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
    

    pc.printf("Starting...\n\r");
    while (1) {

        t.reset();
                    
        // get data from TOF sensor and encoder
        t2.reset();
        // reading all of the continuous range measurements takes about 2ms with current wait times
        set_mux1(2);
        wait_us(10);
        range[0] = tof1.readRangeResult();
        wait_us(10);
        range_status[0] = tof1.readRangeStatus();
        wait_us(10);
        set_mux1(3);
        wait_us(10);
        range[1] = tof2.readRangeResult();
        wait_us(10);
        range_status[1] = tof2.readRangeStatus();
        wait_us(10);
        set_mux1(4);
        wait_us(10);
        range[2] = tof3.readRangeResult();
        wait_us(10);
        range_status[2] = tof3.readRangeStatus();
        wait_us(10);
        samp1 = t2.read_us();

        t2.reset();
        set_mux2(2);
        wait_us(10);
        range[3] = tof4.readRangeResult();
        wait_us(10);
        range_status[3] = tof4.readRangeStatus();
        wait_us(10);
        set_mux2(3);
        wait_us(10);
        range[4] = tof5.readRangeResult();
        wait_us(10);
        range_status[4] = tof5.readRangeStatus();
        wait_us(10);
        set_mux2(4);
        wait_us(10);
        range[5] = tof6.readRangeResult();
        wait_us(10);
        range_status[5] = tof6.readRangeStatus();
        wait_us(10);

        samp2 = t2.read_us();
//        t2.reset();
//        light = tof.readLux(VL6180X_ALS_GAIN_5);
//        samp2 = t2.read_us();
        
        
    //    old_enc_pos = new_enc_pos;
    //    cs = 0;
    //    new_enc_pos = spi.write(0xFFFF);
    //    cs = 1;
    //    new_enc_pos = (new_enc_pos>>3);
    //    if ((new_enc_pos>3000)&&(old_enc_pos<1000)){
    //        num_revolutions -= 1;
    //    } else if ((new_enc_pos<1000)&&(old_enc_pos>3000)){
    //        num_revolutions += 1;
    //    }
    //    position = -(((float)new_enc_pos)*counts_to_mm + ((float)num_revolutions)*revs_to_mm - starting_position);
        
        // print data
        // pc.printf("%f, %d\n\r", t3.read(), range);
//        pc.printf("%d, %f, %d, %d\n\r", range, light, samp1, samp2);
//        pc.printf("prox: %d, raw_enc: %d, pos: %.2f\n\r", prox, new_enc_pos, position); //new_enc_pos); //(enc_data>>3));
        // pc.printf("%f, %d, %.2f\n\r", t3.read(), range, position);
        t2.reset();
        // printing data takes about 900us at baud of 460800
        // pc.printf("%.2f, %d, %d, %d\n\r",t3.read(), samp1, samp2, samp3);
        pc.printf("%.3f, %d, %d, %d, %d, %d, %d\n\r\n\r",t3.read(), range[0], range[1], range[2], range[3], range[4], range[5]);
        // pc.printf("%d, %d, %d, %d, %d, %d\n\r\n\r", range_status[0], range_status[1], range_status[2], range_status[3], range_status[4], range_status[5]);                  
        samp3 = t2.read_us();
        wait_us(10);

        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}