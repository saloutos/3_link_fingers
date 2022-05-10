// controlling two 3-link fingers with lab force sensors and 9 total time-of-flight sensors

#include "mbed.h"
#include "string.h"
#include "VL6180X.h"
#include "ForceSensor.h"
#include "bmp3_funcs.h"
#include "bmp3.h"
#include "neural_nets.h"
#include "math_ops.h"

#define REST_MODE           0
#define PRINT_NN_MODE       1
#define PRINT_RAW_MODE      2
#define PRINT_TOF_MODE      3
#define CAN_MODE            4

#define CAN_ID              3
#define CAN_FORCE_1         9
#define CAN_FORCE_2         10
#define CAN_TOF_1           11
#define CAN_TOF_2           12

// new limits for force sensors, need to test these!
#define FT_MIN -20.0f
#define FT_MAX 20.0f 
#define FN_MIN -30.0f
#define FN_MAX 30.0f
#define ANG_MIN -45.0f
#define ANG_MAX 45.0f
#define RNG_MAX 255 // this probably won't be necessary


Serial pc(USBTX, USBRX, 921600); //460800);
DigitalOut led(LED1);

float loop_time = 0.01f; // 200hz is probably maximum sample rate since that is pressure sensor rate too
// full loop takes about 10ms, so 100Hz is maximum achievable rate with everything on one board

// Set up CAN
CAN can(PB_8, PB_9, 1000000);
CANMessage rxMsg;
CANMessage txMsg_t1, txMsg_t2, txMsg_f1, txMsg_f2; // ToF and force for each finger


// Left finger force sensor
SPI spi1(PB_15, PB_14, PB_13); // MOSI, MISO, SCK
DigitalOut cs01(PC_9);
DigitalOut cs11(PA_6);
DigitalOut cs21(PA_7); 
DigitalOut cs31(PB_6); 
DigitalOut cs41(PA_9);
DigitalOut cs51(PB_10); 
DigitalOut cs61(PB_5);
DigitalOut cs71(PA_10);
extern NeuralNet sensor31;
ForceSensor left_finger(1, &sensor31); // class is modified to use new digital outs and spi

// Right finger force sensor
SPI spi2(PC_12, PC_11, PC_10); // MOSI, MISO, SCK
DigitalOut cs02(PC_8);
DigitalOut cs12(PC_5);
DigitalOut cs22(PA_12); 
DigitalOut cs32(PA_11); 
DigitalOut cs42(PB_12);
DigitalOut cs52(PB_2); 
DigitalOut cs62(PB_1);
DigitalOut cs72(PC_4);
extern NeuralNet sensor28;
ForceSensor right_finger(2, &sensor28); // class is modified to use new digital outs and spi

// ToF sensor i2c busses
I2C i2c1(PC_7, PC_6); // SDA, SCL
I2C i2c2(PB_4, PA_8); 

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

// DigitalOut xs1(PF_5); // reset for sensor 1
// DigitalOut xs2(PF_4);
// DigitalOut xs3(PE_8);
// DigitalOut xs4(PF_10);
// DigitalOut xs5(PE_7);
// DigitalOut xs6(PD_14);
// DigitalOut xs7(PD_15);
// DigitalOut xs8(PF_14);
// DigitalOut xs9(PE_9);

int range[9];
float range_m[9]; // range in m
// TODO: remove some of this, do filtering upstream?
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

// setup for state machine and sensor interrupts
volatile int state = REST_MODE;
volatile int send_data_flag = 0;
Timer timer;
float cur_time;
Ticker send_data;
void send_new_data(){
    send_data_flag = 1;
} 

/// Force Sensor CAN Reply Packet Structure ///
/// 12 bit force in x
/// 12 bit force in y
/// 12 bit force in z
/// 12 bit angle theta
/// 12 bit angle phi
/// CAN packet is 6 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: channel ID
/// 1: [fx[7-0]] // not correct!
/// 2: [fy[7-0]] 
/// 3: [fz[7-0]] 
/// 4: [theta[7-0]] 
/// 5: [phi[7-0]] 

void pack_force_reply(CANMessage * msg, ForceSensor * fs){
     
     /// limit data to be within bounds ///
     float fx_temp = fminf(fmaxf(FT_MIN, fs->output_data[0]), FT_MAX);
     float fy_temp = fminf(fmaxf(FT_MIN, fs->output_data[1]), FT_MAX);   
     float fz_temp = fminf(fmaxf(FN_MIN, fs->output_data[2]), FN_MAX);   
     float theta_temp = fminf(fmaxf(ANG_MIN, fs->output_data[3]), ANG_MAX);   
     float phi_temp = fminf(fmaxf(ANG_MIN, fs->output_data[4]), ANG_MAX);                       
     /// convert floats to unsigned ints ///
     uint16_t fx_int = float_to_uint(fx_temp, FT_MIN, FT_MAX, 12); 
     uint16_t fy_int = float_to_uint(fy_temp, FT_MIN, FT_MAX, 12);  
     uint16_t fz_int = float_to_uint(fz_temp, FN_MIN, FN_MAX, 12);  
     uint16_t theta_int = float_to_uint(theta_temp, ANG_MIN, ANG_MAX, 12);  
     uint16_t phi_int = float_to_uint(phi_temp, ANG_MIN, ANG_MAX, 12);             
     /// pack ints into the can buffer ///
     msg->data[0] = (fs->_channel<<4)|(fx_int>>8);                                       
     msg->data[1] = fx_int&0xFF;
     msg->data[2] = fy_int>>4;
     msg->data[3] = ((fy_int&0x0F)<<4)|(fz_int>>8);
     msg->data[4] = fz_int&0xFF;
     msg->data[5] = theta_int>>4;
     msg->data[6] = ((theta_int&0x0F)<<4)|(phi_int>>8);
     msg->data[7] = phi_int&0xFF;
     }


/// ToF Sensor CAN Reply Packet Structure ///
/// 5 x 8bit range measurements
/// CAN packet is 6 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: finger ID (left is 0, right is 1)
/// 1: [tof[7-0]] 
/// 2: [tof[7-0]] 
/// 3: [tof[7-0]] 
/// 4: [tof[7-0]] 
/// 5: [tof[7-0]]
/// left finger is sensors 1,2,3,4,5
/// right finger is sensors 6,7,8,9,~ 

void pack_tof_reply(CANMessage * msg, uint8_t finger){

     int id_off = 0;
     if (finger==1){
         id_off = 6;
     } 
     /// pack ints into the can buffer ///
     msg->data[0] = finger;                                       
     msg->data[1] = range[0+id_off];
     msg->data[2] = range[1+id_off];
     msg->data[3] = range[2+id_off];
     msg->data[4] = range[3+id_off];
     if (finger==0){
         msg->data[5] = range[4];
     } else {
         msg->data[5] = 0;
     }

     }


// CAN RX interrupt
void onMsgReceived() {
 
    can.read(rxMsg);  
    if(rxMsg.id == CAN_ID){
        
        // Enable message
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC))){
            state = CAN_MODE;
            pc.printf("Entering CAN mode.\n\r");
            }
        // Disable message
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD))){
            state = REST_MODE;
            pc.printf("Entering rest mode.\n\r");
            }
        }
}

void enter_menu_state(void){
    
    // TODO: make more useful menu
    printf("\n\r\n\r\n\r");
    printf(" Commands:\n\r");
    wait_us(10);
    printf(" e - Enter CAN Mode\n\r");
    wait_us(10);
    printf(" n - Print Force Outputs\n\r");
    wait_us(10);
    printf(" r - Print Raw Sensor Values\n\r");
    wait_us(10);
    printf(" t - Pring Tof Values\n\r");
    wait_us(10);
    printf(" esc - Exit to Menu\n\r");
    wait_us(10);
    }
    
    
/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    while(pc.readable()){
        char c = pc.getc();
        if(c == 27){
            state = REST_MODE;
            led = 0;
            enter_menu_state();
        }
        if(state == REST_MODE){
            switch (c){
                case 'e':
                    state = CAN_MODE; // enabled for CAN
                    pc.printf("Entering CAN mode.\n\r");
//                    led = 1;
                    break;
                case 'n':
                    state = PRINT_NN_MODE;
                    pc.printf("Printing NN outputs.\n\r");
//                    led = 1;
                    break;
                case 'r':
                    state = PRINT_RAW_MODE;
                    pc.printf("Printing raw outputs.\n\r");
//                    led = 1;
                    break;
                case 't':
                    state = PRINT_TOF_MODE;
                    pc.printf("Printing raw outputs.\n\r");
//                    led = 1;
                    break;
                }
        }  
    }
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
    // xs1 = 0;
    wait_us(10000);
    // xs1 = 1;
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
    // xs2 = 0;
    wait_us(10000);
    // xs2 = 1;
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
    // xs3 = 0;
    wait_us(10000);
    // xs3 = 1;
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
    // xs4 = 0;
    wait_us(10000);
    // xs4 = 1;
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
    set_mux2(6); // channel 6 on mux 2
    wait_us(1000);
    // xs5 = 0;
    wait_us(10000);
    // xs5 = 1;
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
    set_mux1(2); // channel 5 on mux 1
    wait_us(1000);
    // xs6 = 0;
    wait_us(10000);
    // xs6 = 1;
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
    set_mux1(3); // channel 2 on mux 1
    wait_us(1000);
    // xs7 = 0;
    wait_us(10000);
    // xs7 = 1;
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
    set_mux1(4); // channel 3 on mux 1
    wait_us(1000);
    // xs8 = 0;
    wait_us(10000);
    // xs8 = 1;
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
    set_mux1(5); // channel 4 on mux 1
    wait_us(1000);
    // xs9 = 0;
    wait_us(10000);
    // xs9 = 1;
    wait_us(10000);
    if(!tof9.begin(&i2c1)){
        pc.printf("Sensor 9 init failed.\n\r");
    }
    wait_us(1000);
    if(tof9.readRangeMode()==0){
        tof9.startRangeContinuous(range_period);
    }
    wait_us(10000);


    // Set up CAN
    can.filter(CAN_ID , 0xFFF, CANStandard, 0);                                                         
    txMsg_f1.id = CAN_FORCE_1;
    txMsg_f2.id = CAN_FORCE_2;
    txMsg_f1.len = 8;
    txMsg_f2.len = 8;
    txMsg_t1.id = CAN_TOF_1;
    txMsg_t2.id = CAN_TOF_2;
    txMsg_t1.len = 6;
    txMsg_t2.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);  





    pc.printf("Starting...\n\r");

    t.reset();
    t.start();
    t2.reset();
    t2.start();
    t3.reset();
    t3.start(); 

    while (1) {


        if(send_data_flag==1){


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
            set_mux2(6);
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
            set_mux1(2);
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
            set_mux1(3);
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
            set_mux1(4);
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
            set_mux1(5);
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
                    range[i] = range[i] + range_offsets[i];
                    if (range_m[i]==0.0f){ //==0.2f
                        range_m[i] = ((float)(range[i]))/1000.0f;

                    } else {
                        range_m[i] = filt_coef*(((float)(range[i]))/1000.0f) + (1.0f-filt_coef)*range_m[i];
                    }
                    range_m_raw[i] = ((float)(range[i]))/1000.0f;
                } else {
                    range[i] = 255;
                    range_m[i] = 0.2f;
                    range_m_raw[i] = 0.2f;
                }
            }    

            // check CAN mode
            if (state==CAN_MODE){
                // pack CAN messages
                pack_force_reply(&txMsg_f1, &left_finger);
                pack_force_reply(&txMsg_f2, &right_finger);
                pack_tof_reply(&txMsg_t1, 0); // left finger
                pack_tof_reply(&txMsg_t2, 1); // right finger
                // write CAN messages
                can.write(txMsg_f1);
                wait_us(100);
                can.write(txMsg_f2);
                wait_us(100);
                can.write(txMsg_t1);
                wait_us(100);
                can.write(txMsg_t2);
                wait_us(100);
            }          
            if (state==PRINT_RAW_MODE){
                // printing raw data from all 3 sensors takes 0.001722 seconds
                pc.printf("%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d \n\r", left_finger.raw_data[0],left_finger.raw_data[1],left_finger.raw_data[2],
                    left_finger.raw_data[3],left_finger.raw_data[4],left_finger.raw_data[5],left_finger.raw_data[6],left_finger.raw_data[7]);  
                pc.printf("%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d \n\r", right_finger.raw_data[0],right_finger.raw_data[1],right_finger.raw_data[2],
                    right_finger.raw_data[3],right_finger.raw_data[4],right_finger.raw_data[5],right_finger.raw_data[6],right_finger.raw_data[7]);   
                pc.printf("\n\r\n\r");
            }
            if (state==PRINT_NN_MODE){
                // printing output data from all 3 sensors takes 0.001415 seconds 
                // print a single line for each sample
                pc.printf("%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n\r",
                    left_finger.output_data[0],left_finger.output_data[1],
                    left_finger.output_data[2],left_finger.output_data[3],left_finger.output_data[4],
                    right_finger.output_data[0],right_finger.output_data[1],right_finger.output_data[2],
                    right_finger.output_data[3],right_finger.output_data[4]); 
                
            }
            if (state==PRINT_TOF_MODE){
                // printing raw data from ToF sensors
                pc.printf("%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d\n\r", range[0], range[1], range[2], range[3], range[4], range[5], range[6], range[7], range[8]);
            }

            samp5 = t.read_us();
            wait_us(10);
            led = !led;

        }
    }
}