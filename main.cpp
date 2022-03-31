// example i2c code for reading proximity data from VCNL36826S sensor

#include "mbed.h"
#include "string.h"
#include "VL6180X.h"
#include "dynamixel_XL330.h"
#include "ForceSensor.h"
#include "bmp3_funcs.h"
#include "bmp3.h"
#include "neural_nets.h"
#include "math_ops.h"

Serial pc(USBTX, USBRX, 460800);
DigitalOut led(LED1);

float loop_time = 0.01f; // 200hz is probably maximum sample rate since that is pressure sensor rate too
// full loop takes about 10ms, so 100Hz is maximum achievable rate with everything on one board

// dynamixels
XL330_bus dxl_bus(1000000, D1, D0, D2); // baud, tx, rx, rts
uint8_t dxl_IDs[] =  {1,2,3,4,5,6}; //in order: Left MCP, Left PIP, Left DIP, Right MCP, Right PIP, Right DIP
int32_t dxl_pos[6];
uint8_t num_IDs = 6;

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
// initialize sensors
VL6180X tof1; // right finger inner
VL6180X tof2; // right finger forward
VL6180X tof3; // right finger outer
VL6180X tof4; // left finger outer
VL6180X tof5; // left finger forward
VL6180X tof6; // left finger inner
VL6180X tof7; // palm
int range[7];
int range_status[7];
uint8_t MUX_ADDR = (0x70<<1);
uint16_t range_period = 30;

// timer stuff
Timer t;
Timer t2;
Timer t3;
int samp0, samp1, samp2, samp3, samp4, samp5;

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
    
    pc.printf("Initializing Dynamixels.\n\r");

    // Enable dynamixels and set control mode...individual version
    for(int i=0; i<6; i++){
        pc.printf("Motor ID %d.\n\r",dxl_IDs[i]);
        dxl_bus.SetTorqueEn(dxl_IDs[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_IDs[i],0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
        wait_us(100);
        dxl_bus.TurnOnLED(dxl_IDs[i], 0x01);
        // dxl_bus.TurnOnLED(dxl_IDs[i], 0x00); // turn off LED
        // dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_us(100);
    } 

    // perform any setup for the sensor here
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
    pc.printf("Sensor 7...\n\r");
    set_mux2(1); // channel 1 on mux
    wait_us(100);
    if(!tof7.begin(&i2c2)){
        pc.printf("Sensor 7 init failed.\n\r");
    }
    wait_us(100);
    tof7.stopRangeContinuous();
    wait_us(100);
    tof7.startRangeContinuous(range_period);
    wait_us(1000);

    left_finger.Initialize();
    left_finger.Calibrate();
    wait_us(10000);
    right_finger.Initialize();
    right_finger.Calibrate();
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
                    
        // get force sensor data
        t2.reset();
        left_finger.Sample();
        left_finger.Evaluate();
        wait_us(10);
        right_finger.Sample();
        right_finger.Evaluate();
        wait_us(10);
        samp0 = t2.read_us();

        // get data from TOF sensor
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
        set_mux2(1);
        wait_us(10);
        range[6] = tof7.readRangeResult();
        wait_us(10);
        range_status[6] = tof7.readRangeStatus();
        wait_us(10);
        samp2 = t2.read_us();

        t2.reset();
        // for(int i=0; i<num_IDs; i++){
        //     dxl_pos[i]= dxl_bus.GetPosition(dxl_IDs[i]);
        // }
        
        dxl_bus.GetMultPositions(dxl_pos, dxl_IDs, num_IDs);
        // convert states
        // for(int i=0; i<num_IDs; i++){
        //     converted_position[i] = (pulse_to_rad*(float)dxl_pos[i])-dxl_offsets[i];
        // }
        samp3 = t2.read_us();

        t2.reset();
        // printing data takes about 900us at baud of 460800
        // pc.printf("%.2f, %d, %d, %d, %d, %d\n\r",t3.read(), samp1+samp2, samp3, samp0, samp4, samp5);
        // pc.printf("%.2f, %d\n\r",t3.read(), samp5);
        pc.printf("%.2f, %d, %d, %d, %d, %d, %d, %d\n\r", t3.read(), range[3], range[4], range[5], range[6], range[0], range[1], range[2]);
        // pc.printf("%d, %d, %d, %d, %d, %d\n\r\n\r", range_status[0], range_status[1], range_status[2], range_status[3], range_status[4], range_status[5]);  
        pc.printf("%d, %d, %d, %d, %d, %d\n\r", dxl_pos[0], dxl_pos[1], dxl_pos[2], dxl_pos[3], dxl_pos[4], dxl_pos[5]);
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f\n\r", left_finger.output_data[0], left_finger.output_data[1], left_finger.output_data[2], left_finger.output_data[3], left_finger.output_data[4]);
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f\n\r\n\r", right_finger.output_data[0], right_finger.output_data[1], right_finger.output_data[2], right_finger.output_data[3], right_finger.output_data[4]);

        samp4 = t2.read_us();
        samp5 = t.read_us();
        wait_us(10);

        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}