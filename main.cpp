// controlling two 3-link fingers with lab force sensors and 9 total time-of-flight sensors

#include "mbed.h"
#include "string.h"
#include "VL6180X.h"
#include "ForceSensor.h"
#include "bmp3_funcs.h"
#include "bmp3.h"
#include "neural_nets.h"
#include "math_ops.h"

Serial pc(USBTX, USBRX, 921600); //460800);
DigitalOut led(LED1);

float loop_time = 0.01f; // 200hz is probably maximum sample rate since that is pressure sensor rate too
// full loop takes about 10ms, so 100Hz is maximum achievable rate with everything on one board

// Left finger force sensor
SPI spi1(PE_6, PE_5, PE_2); // MOSI, MISO, SCK
DigitalOut cs01(PD_4);
DigitalOut cs11(PD_5);
DigitalOut cs21(PD_6); 
DigitalOut cs31(PD_7); 
DigitalOut cs41(PD_1);
DigitalOut cs51(PD_0); 
DigitalOut cs61(PG_0);
DigitalOut cs71(PG_12);
extern NeuralNet sensor31;
ForceSensor left_finger(1, &sensor31); // class is modified to use new digital outs and spi

// Right finger force sensor
SPI spi2(PF_9, PF_8, PF_7); // MOSI, MISO, SCK
DigitalOut cs02(PF_13);
DigitalOut cs12(PF_12);
DigitalOut cs22(PD_10); 
DigitalOut cs32(PG_7); 
DigitalOut cs42(PG_4);
DigitalOut cs52(PG_5); 
DigitalOut cs62(PG_8);
DigitalOut cs72(PF_11);
// initialize neural net structs
extern NeuralNet sensor28;
ForceSensor right_finger(2, &sensor28); // class is modified to use new digital outs and spi

// ToF sensor i2c busses
I2C i2c1(PF_0, PF_1); //(PB_9, PB_8); // SDA, SCL
I2C i2c2(PD_13, PD_12); 

// initialize ToF sensors
VL6180X tof1; // left finger outer =range[0]
VL6180X tof2; // left finger forward
VL6180X tof3; // left finger inner distal
VL6180X tof4; // left finger inner proximal
VL6180X tof5; // palm
VL6180X tof6; // right finger inner proximal
VL6180X tof7; // right finger inner distal
VL6180X tof8; // right finger forward
VL6180X tof9; // right finger outer

DigitalOut xs1(PF_5); // reset for sensor 1
DigitalOut xs2(PF_4);
DigitalOut xs3(PE_8);
DigitalOut xs4(PF_10);
DigitalOut xs5(PE_7);
DigitalOut xs6(PD_14);
DigitalOut xs7(PD_15);
DigitalOut xs8(PF_14);
DigitalOut xs9(PE_9);

int range[9];
float range_m[9]; // range in m
float range_m_raw[9]; // range in m, non-filtered
int range_status[9];
int range_mode[9];
int range_offsets[] = {4, 8, 9, 10, 5, 0, 0, 10, 3}; // offsets in mm to be added to range measurements
uint8_t MUX_ADDR = (0x70<<1);
uint16_t range_period = 30;
float filt_coef = 0.70f;

// timer stuff
Timer t;
Timer t2;
Timer t3;
int samp0, samp1, samp2, samp3, samp4, samp5;

// Helper functions for i2c
void set_mux1(uint8_t channel){
    // write to mux address
    char buffer[1];
    buffer[0] = (1 << channel);
    int result = 1; 
    result = i2c1.write(MUX_ADDR, buffer, 1);
    // pc.printf("M: 1, C: %d, r: %d\n\r", channel, result);
}

void set_mux2(uint8_t channel){
    // write to mux address
    char buffer[1];
    buffer[0] = (1 << channel);
    int result = 1; 
    result = i2c2.write(MUX_ADDR, buffer, 1);
    // pc.printf("M: 2, C: %d, r: %d\n\r", channel, result);
}

// main loop
int main() {
    
    // start up

    // wait(3.0);
    wait_us(50000);

    pc.printf("Initializing.\n\r");
    
    i2c1.frequency(400000); // set bus freq
    i2c2.frequency(400000); 
    
    left_finger.Initialize();
    left_finger.Calibrate();
    wait_us(10000);
    right_finger.Initialize();
    right_finger.Calibrate();
    wait_us(10000);

    // perform any setup for the sensor here
    pc.printf("Sensor 1...\n\r");
    set_mux2(2); // channel 2 on mux 2 
    wait_us(1000);
    xs1 = 0;
    wait_us(10000);
    xs1 = 1;
    wait_us(10000);
    if(!tof1.begin(&i2c2)){
        pc.printf("Sensor 1 init failed.\n\r");
    }
    wait_us(1000);
    pc.printf("Range mode: %d\n\r",tof1.readRangeMode());
    if(tof1.readRangeMode()==0){ // TODO: might be able to remove this check since we have the reset lines now
        tof1.startRangeContinuous(range_period);
    }
    wait_us(10000);
    pc.printf("Sensor 2...\n\r");
    set_mux2(3); // channel 3 on mux 2
    wait_us(1000);
    xs2 = 0;
    wait_us(10000);
    xs2 = 1;
    wait_us(10000);
    if(!tof2.begin(&i2c2)){
        pc.printf("Sensor 2 init failed.\n\r");
    }
    wait_us(1000);
    pc.printf("Range mode: %d\n\r",tof2.readRangeMode());
    if(tof2.readRangeMode()==0){
        tof2.startRangeContinuous(range_period);
    }
    wait_us(10000);


    pc.printf("Sensor 3...\n\r");
    set_mux2(4); // channel 4 on mux 2
    wait_us(1000);
    xs3 = 0;
    wait_us(10000);
    xs3 = 1;
    wait_us(10000);
    if(!tof3.begin(&i2c2)){
        pc.printf("Sensor 3 init failed.\n\r");
    }
    wait_us(1000);
    if(tof3.readRangeMode()==0){
        tof3.startRangeContinuous(range_period);
    }
    wait_us(10000);
    pc.printf("Sensor 4...\n\r");
    set_mux2(5); // channel 5 on mux 2 
    wait_us(1000);
    xs4 = 0;
    wait_us(10000);
    xs4 = 1;
    wait_us(10000);
    if(!tof4.begin(&i2c2)){
        pc.printf("Sensor 4 init failed.\n\r");
    }
    wait_us(1000);
    if(tof4.readRangeMode()==0){
        tof4.startRangeContinuous(range_period);
    }
    wait_us(10000);


    pc.printf("Sensor 5...\n\r");
    set_mux2(1); // channel 1 on mux 2
    wait_us(1000);
    xs5 = 0;
    wait_us(10000);
    xs5 = 1;
    wait_us(10000);
    if(!tof5.begin(&i2c2)){
        pc.printf("Sensor 5 init failed.\n\r");
    }
    wait_us(1000);
    if(tof5.readRangeMode()==0){
        tof5.startRangeContinuous(range_period);
    }
    wait_us(10000);


    pc.printf("Sensor 6...\n\r");
    set_mux1(5); // channel 5 on mux 1
    wait_us(1000);
    xs6 = 0;
    wait_us(10000);
    xs6 = 1;
    wait_us(10000);
    if(!tof6.begin(&i2c1)){
        pc.printf("Sensor 6 init failed.\n\r");
    }
    wait_us(1000);
    if(tof6.readRangeMode()==0){
        tof6.startRangeContinuous(range_period);
    }
    wait_us(10000);
    pc.printf("Sensor 7...\n\r");
    set_mux1(2); // channel 2 on mux 1
    wait_us(1000);
    xs7 = 0;
    wait_us(10000);
    xs7 = 1;
    wait_us(10000);
    if(!tof7.begin(&i2c1)){
        pc.printf("Sensor 7 init failed.\n\r");
    }
    wait_us(1000);
    if(tof7.readRangeMode()==0){
        tof7.startRangeContinuous(range_period);
    }
    wait_us(10000);


    pc.printf("Sensor 8...\n\r");
    set_mux1(3); // channel 3 on mux 1
    wait_us(1000);
    xs8 = 0;
    wait_us(10000);
    xs8 = 1;
    wait_us(10000);
    if(!tof8.begin(&i2c1)){
        pc.printf("Sensor 8 init failed.\n\r");
    }
    wait_us(1000);
    if(tof8.readRangeMode()==0){
        tof8.startRangeContinuous(range_period);
    }
    wait_us(10000);
    pc.printf("Sensor 9...\n\r");
    set_mux1(4); // channel 4 on mux 1
    wait_us(1000);
    xs9 = 0;
    wait_us(10000);
    xs9 = 1;
    wait_us(10000);
    if(!tof9.begin(&i2c1)){
        pc.printf("Sensor 9 init failed.\n\r");
    }
    wait_us(1000);
    if(tof9.readRangeMode()==0){
        tof9.startRangeContinuous(range_period);
    }
    wait_us(10000);


    pc.printf("Starting...\n\r");

    t.reset();
    t.start();
    t2.reset();
    t2.start();
    t3.reset();
    t3.start(); 

    while (1) {

        t.reset();
        t2.reset();                    
        
        // get force sensor data
        left_finger.Sample();
        left_finger.Evaluate();
        wait_us(10);
        right_finger.Sample();
        right_finger.Evaluate();
        wait_us(10);
        samp0 = t2.read_us();

        // get data from TOF sensor
        t2.reset(); // reading all of the continuous range measurements takes about 2ms with current wait times

        // TODO: could remove range status reading to speed up?
        set_mux2(2);
        if (tof1.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[0] = tof1.readRangeStatus();
            // wait_us(10);
            // range_mode[0] = tof1.readRangeMode();
            wait_us(10);
            range[0] = tof1.readRangeResult();
            wait_us(10);
        }
        set_mux2(3);
        if (tof2.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[1] = tof2.readRangeStatus();
            // wait_us(10);
            // range_mode[1] = tof2.readRangeMode();
            wait_us(10);
            range[1] = tof2.readRangeResult();
            wait_us(10);
        }
        set_mux2(4);
        if (tof3.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[2] = tof3.readRangeStatus();
            // wait_us(10);
            // range_mode[2] = tof3.readRangeMode();
            wait_us(10);
            range[2] = tof3.readRangeResult();
            wait_us(10);
        }
        set_mux2(5);
        if (tof4.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[3] = tof4.readRangeStatus();
            // wait_us(10);
            // range_mode[3] = tof4.readRangeMode();
            wait_us(10);
            range[3] = tof4.readRangeResult();
            wait_us(10);
        }
        set_mux2(1);
        if (tof5.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[4] = tof5.readRangeStatus();
            // wait_us(10);
            // range_mode[4] = tof5.readRangeMode();
            wait_us(10);
            range[4] = tof5.readRangeResult();
            wait_us(10);
        }

        samp1 = t2.read_us();

        t2.reset();
        set_mux1(5);
        if (tof6.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[5] = tof6.readRangeStatus();
            // wait_us(10);
            // range_mode[5] = tof6.readRangeMode();
            wait_us(10);
            range[5] = tof6.readRangeResult();
            wait_us(10);
        }
        set_mux1(2);
        if (tof7.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[6] = tof7.readRangeStatus();
            // wait_us(10);
            // range_mode[6] = tof7.readRangeMode();
            wait_us(10);
            range[6] = tof7.readRangeResult();
            wait_us(10);
        }
        set_mux1(3);
        if (tof8.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[7] = tof8.readRangeStatus();
            // wait_us(10);
            // range_mode[7] = tof8.readRangeMode();
            wait_us(10);
            range[7] = tof8.readRangeResult();
            wait_us(10);
        }
        set_mux1(4);
        if (tof9.isRangeComplete()){
            // if it is ready, get range status, mode, and result
            // wait_us(10);
            // range_status[8] = tof9.readRangeStatus();
            // wait_us(10);
            // range_mode[8] = tof9.readRangeMode();
            wait_us(10);
            range[8] = tof9.readRangeResult();
            wait_us(10);
        }
        samp2 = t2.read_us();

        // convert range measurements to meters and do some filtering
        for(int i=0; i<9; i++){
            if (range_status[i]==0){ // good measurement
                if (range_m[i]==0.0f){ //==0.2f
                    range_m[i] = ((float)(range[i]+range_offsets[i]))/1000.0f;

                } else {
                    range_m[i] = filt_coef*(((float)(range[i]+range_offsets[i]))/1000.0f) + (1.0f-filt_coef)*range_m[i];
                }
                range_m_raw[i] = ((float)(range[i]+range_offsets[i]))/1000.0f;
            } else {
                range_m[i] = 0.2f;
                range_m_raw[i] = 0.2f;
            }
        }    

        t2.reset();
        
        pc.printf("%.2f, %1.3f, %1.3f,    %1.4f, %1.4f,    %1.3f,    %1.4f, %1.4f,   %1.3f, %1.3f\n\r", t3.read(), range_m[0], range_m[1], range_m[2], range_m[3], range_m[4], range_m[5], range_m[6], range_m[7], range_m[8]);

        samp4 = t2.read_us();
        samp5 = t.read_us();
        wait_us(10);

        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}