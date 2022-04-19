// controlling two 3-link fingers with lab force sensors and 9 total time-of-flight sensors

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

float loop_time = 0.05f; // 200hz is probably maximum sample rate since that is pressure sensor rate too
// full loop takes about 10ms, so 100Hz is maximum achievable rate with everything on one board

// dynamixels
XL330_bus dxl_bus(1000000, D1, D0, D2); // baud, tx, rx, rts
uint8_t dxl_IDs[] =  {1,2,3,4,5,6}; //in order: Left MCP, Left PIP, Left DIP, Right MCP, Right PIP, Right DIP
float dxl_offsets[] = {3.916f, 3.898f, 3.901f, 5.513f, 2.304f, 2.404f};
int32_t dxl_pos[6];
uint8_t num_IDs = 6;
float pulse_to_rad = (2.0f*3.14159f)/4096.0f; // = 0.001534
float conv_pos[6];
float default_pos[] = {1.0f, -1.8f, 1.3f, -1.0f, 1.8f, -1.3f}; // nominal joint positions
// float default_pos[] = {0.9f, -0.9f, -0.7f, -0.9f, 0.9f, 0.7f}; // nominal joint positions
float link3_angles_des[] = {0.5f, -0.5f}; // [left,right] in world reference frame
float new_pos[6];
uint32_t des_pos[6];
uint16_t des_cur[6];

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
VL6180X tof1; // left finger outer
VL6180X tof2; // left finger forward
VL6180X tof3; // left finger inner distal
VL6180X tof4; // left finger inner proximal
VL6180X tof5; // palm
VL6180X tof6; // right finger inner proximal
VL6180X tof7; // right finger inner distal
VL6180X tof8; // right finger forward
VL6180X tof9; // right finger outer

int range[9];
float range_m[9]; // range in m
float range_m_raw[9]; // range in m, non-filtered
int range_status[9];
uint8_t MUX_ADDR = (0x70<<1);
uint16_t range_period = 20;

// timer stuff
Timer t;
Timer t2;
Timer t3;
int samp0, samp1, samp2, samp3, samp4, samp5;

// finger kinematics
float l1 = 0.04f;
float l2 = 0.05f;
float l3 = 0.03f;
float lout = 0.0125f;
float lfw = 0.015f;
float lin = 0.0125f;
float linbw = 0.0225;
float l_yoff = 0.0475f;
float r_yoff = -0.0475f;
float c_xoff = -0.025f;
float p[2][3];
float v[2][3];
float etip_left[3][2]; // out, fw, in
float etip_right[3][2]; // out, fw, in
float J_left[3][3]; // 
float JT_left[3][3]; // 
float tau_left[3][1];
float des_left[3][1];
float J_right[3][3]; // 
float JT_right[3][3]; // 
float tau_right[3][1];
float des_right[3][1];
float pstar[2][2]; // desired change in end-effector position
float prox_thresh = 0.030f; // avoidance threshold
float glide_thresh = 0.060f; // glide threshold
float corr_thresh = 0.080f; // link 3 correction threshold
float ang_thresh = 0.0f; // angle threshold, in mm difference
float ang_max = 1.5; // maximum angle correction from nominal
float ang_min = -1.5; // minimum angle correction from nominal
float glide_dist = 0.030f; // desired contact distance for gliding/contour following
float link3_delta_limit = 0.2f; // maximum change in position command
int left_ik = 0;
int right_ik = 0;
float link3_corrections[2]; // corrections to link 3 angles based on difference in proximity measurements
float x2, y2, l4, gamma, alpha1, alpha2;  
float pos_eps = 0.3; // exponential return to the default pose
float l_diff, l_avg, r_diff, r_avg; // values for internal sensors

// takes joint angles and populates finger kinematics x,y,theta
// void kinematics(){
//
// }

// takes finger number, finger pose (x,y,theta) and returns corresponding joint angles
// void inverse_kinematics(){
// 
// }

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

void mult33x31(float Rout[3][1], float R1[3][3], float R2[3][1]){
    // multiply 3x3 by 3x1
    Rout[0][0] = R1[0][0]*R2[0][0] + R1[0][1]*R2[1][0] + R1[0][2]*R2[2][0];
    Rout[1][0] = R1[1][0]*R2[0][0] + R1[1][1]*R2[1][0] + R1[1][2]*R2[2][0];
    Rout[2][0] = R1[2][0]*R2[0][0] + R1[2][1]*R2[1][0] + R1[2][2]*R2[2][0];
}

// main loop
int main() {
    
    // start up

    // wait(3.0);


    pc.printf("Initializing.\n\r");
    
    i2c1.frequency(400000); // set bus freq
    i2c2.frequency(400000); 
    
    pc.printf("Initializing Dynamixels.\n\r");

    // Enable dynamixels and set control mode...individual version
    for(int i=0; i<6; i++){
        pc.printf("Motor ID %d.\n\r",dxl_IDs[i]);
        dxl_bus.SetTorqueEn(dxl_IDs[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_IDs[i],0x32); // 4us delay time?
        //dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
        dxl_bus.SetControlMode(dxl_IDs[i], CURRENT_CONTROL);
        wait_us(100);
        // dxl_bus.TurnOnLED(dxl_IDs[i], 0x01);
        // dxl_bus.TurnOnLED(dxl_IDs[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_IDs[i], 0x01); //to be able to move 
        wait_us(100);
    } 

    // perform any setup for the sensor here
    pc.printf("Sensor 1...\n\r");
    set_mux2(2); // channel 2 on mux 2 
    wait_us(1000);
    if(!tof1.begin(&i2c2)){
        pc.printf("Sensor 1 init failed.\n\r");
    }
    // wait_us(1000);
    // tof1.stopRangeContinuous();
    // wait_us(1000);
    // tof1.startRangeContinuous(range_period);
    wait_us(10000);
    pc.printf("Sensor 2...\n\r");
    set_mux2(3); // channel 3 on mux 2
    wait_us(1000);
    if(!tof2.begin(&i2c2)){
        pc.printf("Sensor 2 init failed.\n\r");
    }
    // wait_us(1000);
    // tof2.stopRangeContinuous();
    // wait_us(1000);
    // tof2.startRangeContinuous(range_period);
    wait_us(10000);


    pc.printf("Sensor 3...\n\r");
    set_mux2(4); // channel 4 on mux 2
    wait_us(1000);
    if(!tof3.begin(&i2c2)){
        pc.printf("Sensor 3 init failed.\n\r");
    }
    // wait_us(1000);
    // tof3.stopRangeContinuous();
    // wait_us(1000);
    // tof3.startRangeContinuous(range_period);
    wait_us(100000);
    pc.printf("Sensor 4...\n\r");
    set_mux2(5); // channel 5 on mux 2 
    wait_us(1000);
    if(!tof4.begin(&i2c2)){
        pc.printf("Sensor 4 init failed.\n\r");
    }
    // wait_us(1000);
    // tof4.stopRangeContinuous();
    // wait_us(1000);
    // tof4.startRangeContinuous(range_period);
    wait_us(10000);


    pc.printf("Sensor 5...\n\r");
    set_mux2(1); // channel 1 on mux 2
    wait_us(1000);
    if(!tof5.begin(&i2c2)){
        pc.printf("Sensor 5 init failed.\n\r");
    }
    // wait_us(1000);
    // tof5.stopRangeContinuous();
    // wait_us(1000);
    // tof5.startRangeContinuous(range_period);
    wait_us(10000);


    pc.printf("Sensor 6...\n\r");
    set_mux1(5); // channel 5 on mux 1
    wait_us(1000);
    if(!tof6.begin(&i2c1)){
        pc.printf("Sensor 6 init failed.\n\r");
    }
    // wait_us(1000);
    // tof6.stopRangeContinuous();
    // wait_us(1000);
    // tof6.startRangeContinuous(range_period);
    wait_us(10000);
    pc.printf("Sensor 7...\n\r");
    set_mux1(2); // channel 2 on mux 1
    wait_us(1000);
    if(!tof7.begin(&i2c1)){
        pc.printf("Sensor 7 init failed.\n\r");
    }
    // wait_us(1000);
    // tof7.stopRangeContinuous();
    // wait_us(1000);
    // tof7.startRangeContinuous(range_period);
    wait_us(10000);


    pc.printf("Sensor 8...\n\r");
    set_mux1(3); // channel 3 on mux 1
    wait_us(1000);
    if(!tof8.begin(&i2c1)){
        pc.printf("Sensor 8 init failed.\n\r");
    }
    // wait_us(1000);
    // tof8.stopRangeContinuous();
    // wait_us(1000);
    // tof8.startRangeContinuous(range_period);
    wait_us(10000);
    pc.printf("Sensor 9...\n\r");
    set_mux1(4); // channel 4 on mux 1
    wait_us(1000);
    if(!tof9.begin(&i2c1)){
        pc.printf("Sensor 9 init failed.\n\r");
    }
    // wait_us(1000);
    // tof9.stopRangeContinuous();
    // wait_us(1000);
    // tof9.startRangeContinuous(range_period);
    wait_us(10000);

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
        // TODO: could remove range status reading to speed up?
        // set_mux2(2);
        // wait_us(10);
        // range[0] = tof1.readRange(); //Result();
        // wait_us(10);
        // range_status[0] = tof1.readRangeStatus();
        // wait_us(10);
        // set_mux2(3);
        // wait_us(10);
        // range[1] = tof2.readRange(); //Result();
        // wait_us(10);
        // range_status[1] = tof2.readRangeStatus();
        // wait_us(10);
        set_mux2(4);
        wait_us(10);
        range[2] = tof3.readRange(); //Result();
        wait_us(10);
        range_status[2] = tof3.readRangeStatus();
        wait_us(10);
        set_mux2(5);
        wait_us(10);
        range[3] = tof4.readRange(); //Result();
        wait_us(10);
        range_status[3] = tof4.readRangeStatus();
        wait_us(10);
        set_mux2(1);
        wait_us(10);
        range[4] = tof5.readRange(); //Result();
        wait_us(10);
        range_status[4] = tof5.readRangeStatus();
        wait_us(10);
        samp1 = t2.read_us();

        t2.reset();
        set_mux1(5);
        wait_us(10);
        range[5] = tof6.readRange(); //Result();
        wait_us(10);
        range_status[5] = tof6.readRangeStatus();
        wait_us(10);
        set_mux1(2);
        wait_us(10);
        range[6] = tof7.readRange(); //Result();
        wait_us(10);
        range_status[6] = tof7.readRangeStatus();
        wait_us(10);
        // set_mux1(3);
        // wait_us(10);
        // range[7] = tof8.readRange(); //Result();
        // wait_us(10);
        // range_status[7] = tof8.readRangeStatus();
        // wait_us(10);
        // set_mux1(4);
        // wait_us(10);
        // range[8] = tof9.readRange(); //Result();
        // wait_us(10);
        // range_status[8] = tof9.readRangeStatus();
        // wait_us(10);
        samp2 = t2.read_us();

        // convert range measurements to meters and do some filtering
        for(int i=0; i<9; i++){
            if (range_status[i]==0){ // good measurement
                if (range_m[i]==0.0f){ //==0.2f
                    range_m[i] = ((float)range[i])/1000.0f;

                } else {
                    range_m[i] = 0.2f*(((float)range[i])/1000.0f) + 0.8f*range_m[i];
                }
                range_m_raw[i] = ((float)range[i])/1000.0f;
            } else {
                range_m[i] = 0.2f;
                range_m_raw[i] = 0.2f;
            }
        }

        t2.reset();        
        dxl_bus.GetMultPositions(dxl_pos, dxl_IDs, num_IDs);
        // convert states
        for(int i=0; i<num_IDs; i++){
            conv_pos[i] = (pulse_to_rad*(float)dxl_pos[i])-dxl_offsets[i];
        }
        samp3 = t2.read_us();

        // evaluate forward kinematics
        p[0][0] = l1*cos(conv_pos[0]) + l2*cos(conv_pos[0]+conv_pos[1]) + l3*cos(conv_pos[0]+conv_pos[1]+conv_pos[2]); // left x
        p[0][1] = l_yoff + l1*sin(conv_pos[0]) + l2*sin(conv_pos[0]+conv_pos[1]) + l3*sin(conv_pos[0]+conv_pos[1]+conv_pos[2]); // left y
        p[0][2] = conv_pos[0]+conv_pos[1]+conv_pos[2]; // left theta
        etip_left[0][0] = -sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[0][1] = cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[1][0] = cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[1][1] = sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[2][0] = sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[2][1] = -cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);

        //Jacobian for Left 
        J_left[0][0] = -l1*sin(conv_pos[0]) -l2*sin(conv_pos[0]+conv_pos[1]) - l3*sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);// dq1/dx with q1=conv_pos[0] or base
        J_left[0][1] = -l2*sin(conv_pos[0]+conv_pos[1]) -l3*sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);// dq2/dx with q2=conv_pos[1]
        J_left[0][2] = -l3*sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);// dq3/dx with q3=conv_pos[2]
        J_left[1][0] =  l1*cos(conv_pos[0]) + l2*cos(conv_pos[0]+conv_pos[1]) + l3*cos(conv_pos[0]+conv_pos[1]+conv_pos[2]); // dq1/dy
        J_left[1][1] =  l2*cos(conv_pos[0]+conv_pos[1]) + l3*cos(conv_pos[0]+conv_pos[1]+conv_pos[2]); // dq2/dy
        J_left[1][2] =  l3*cos(conv_pos[0]+conv_pos[1]+conv_pos[2]); // dq3/dy
        J_left[2][0] = 1.0f;
        J_left[2][1] = 1.0f;
        J_left[2][2] = 1.0f;

        // evaluate forward kinematics
        p[1][0] = l1*cos(conv_pos[3]) + l2*cos(conv_pos[3]+conv_pos[4]) + l3*cos(conv_pos[3]+conv_pos[4]+conv_pos[5]); // right x
        p[1][1] = r_yoff + l1*sin(conv_pos[3]) + l2*sin(conv_pos[3]+conv_pos[4]) + l3*sin(conv_pos[3]+conv_pos[4]+conv_pos[5]); // right y
        p[1][2] = conv_pos[3]+conv_pos[4]+conv_pos[5]; // right theta
        etip_right[0][0] = sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[0][1] = -cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[1][0] = cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[1][1] = sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[2][0] = -sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[2][1] = cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);

        //Jacobian for right 
        J_right[0][0] = -l1*sin(conv_pos[3]) -l2*sin(conv_pos[3]+conv_pos[4]) - l3*sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);// dq1/dx with q1=conv_pos[0] or base
        J_right[0][1] = -l2*sin(conv_pos[3]+conv_pos[4]) -l3*sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);// dq2/dx with q2=conv_pos[1]
        J_right[0][2] = -l3*sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);// dq3/dx with q3=conv_pos[2]
        J_right[1][0] =  l1*cos(conv_pos[3]) + l2*cos(conv_pos[3]+conv_pos[4]) + l3*cos(conv_pos[3]+conv_pos[4]+conv_pos[5]); // dq1/dy
        J_right[1][1] =  l2*cos(conv_pos[3]+conv_pos[4]) + l3*cos(conv_pos[3]+conv_pos[4]+conv_pos[5]); // dq2/dy
        J_right[1][2] =  l3*cos(conv_pos[3]+conv_pos[4]+conv_pos[5]); // dq3/dy
        J_right[2][0] = 1.0f;
        J_right[2][1] = 1.0f;
        J_right[2][2] = 1.0f;

        //Jacobian transpose for both left and right   
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                JT_right[i][j]=J_right[j][i];
                JT_left[i][j]=J_left[j][i];
            }
        }

        // check sensor values against proximity threshold
        // if threshold is violated, calculate necessary inverse kinematics solution
        pstar[0][0] = p[0][0]; // reset pstar and ik flags
        pstar[0][1] = p[0][1];
        pstar[1][0] = p[1][0];
        pstar[1][1] = p[1][1];
        left_ik = 0;
        right_ik = 0;

        // Calculate "gliding" movements for link 3 angles and maintaining a distance from contact
        // for inner sensors, use difference in values to balance link 3 and average of values to move tip
        l_diff = 0.0f;
        l_avg = 0.0f;
        r_diff = 0.0f;
        r_avg = 0.0f;
        if ((range_m[2]<corr_thresh)||(range_m[3]<corr_thresh)){ // if at least one of the sensors is activated for angle correction
            l_diff = fmaxf2( fminf2( range_m[3]-range_m[2], corr_thresh), -corr_thresh);
            if (abs(l_diff)>ang_thresh){
                link3_corrections[0] = atan2(l_diff,linbw);
                link3_corrections[0] = fmaxf2( fminf2( link3_corrections[0], ang_max), ang_min);
            }
        }  else {
            link3_corrections[0] = link3_angles_des[0];
        }
        if ((range_m[2]<glide_thresh)&&(range_m[3]<glide_thresh)){ // if both sensors are activated for gliding 
            left_ik = 1;
            l_avg = 0.5*(range_m_raw[2]+range_m_raw[3]);
            pstar[0][0] += (l_avg-glide_dist)*etip_left[2][0]; // x
            pstar[0][1] += (l_avg-glide_dist)*etip_left[2][1]; // y
        }
        if ((range_m[5]<corr_thresh)||(range_m[6]<corr_thresh)){ // if at least one of the sensors is activated for angle correction
            r_diff = fmaxf2( fminf2( range_m[6]-range_m[5], corr_thresh), -corr_thresh);
            if (abs(r_diff)>ang_thresh){
                link3_corrections[1] = atan2(r_diff,linbw);
                link3_corrections[1] = fmaxf2( fminf2( link3_corrections[1], ang_max), ang_min);
            }
        } else {
            link3_corrections[1] = link3_angles_des[1];
        }
        if ((range_m[5]<glide_thresh)&&(range_m[6]<glide_thresh)){ // if both sensors are activated for gliding
            right_ik = 1;
            r_avg = 0.5*(range_m_raw[5]+range_m_raw[6]);
            pstar[1][0] += (r_avg-glide_dist)*etip_right[2][0]; // x
            pstar[1][1] += (r_avg-glide_dist)*etip_right[2][1]; // y
        }


        // // Calculate "avoidance" movements for outer sensors
        // for (int i=0; i<2; i++){ // left finger sensors: (out,fw) range[0], range[1]
        //     if (range_m[i]<prox_thresh){
        //         left_ik = 1;
        //         pstar[0][0] += (range_m[i]-prox_thresh)*etip_left[i][0]; // x
        //         pstar[0][1] += (range_m[i]-prox_thresh)*etip_left[i][1]; // y
        //     }
        // }
        // for (int i=0; i<2; i++){ // right finger sensors: (out,fw) range[8], range[7]
        //     if (range_m[8-i]<prox_thresh){
        //         right_ik = 1;
        //         pstar[1][0] += (range_m[8-i]-prox_thresh)*etip_right[i][0]; // x
        //         pstar[1][1] += (range_m[8-i]-prox_thresh)*etip_right[i][1]; // y
        //     }
        // }

        // use inverse kinematics to calculate new joint positions (in radians)
        // TODO: make inverse kinematics more robust! (use received angles for initial calculations?)

        //Comment out old IK
        /*
        if (left_ik==1){
            // calculate ik based on pstar
            // link3_corrections[0] = link3_corrections[0] - conv_pos[0] - conv_pos[1]; // convert to joint angle to limit movement
            // if ((link3_corrections[0]-conv_pos[2])>link3_delta_limit){
            //     link3_corrections[0] = conv_pos[2] + link3_delta_limit;
            // } else if ((link3_corrections[0]-conv_pos[2])<-link3_delta_limit){
            //     link3_corrections[0] = conv_pos[2] - link3_delta_limit;
            // }
            // link3_corrections[0] = link3_corrections[0] + conv_pos[0] + conv_pos[1]; // convert back to world angle
            x2 = pstar[0][0] - l3*cos(link3_corrections[0]);
            y2 = pstar[0][1] - l3*sin(link3_corrections[0]) - l_yoff;
            l4 = fminf2( sqrt(x2*x2 + y2*y2), l1+l2);
            gamma = atan2(y2,x2);
            alpha1 = acos(((l1*l1+l4*l4-l2*l2)/(2.0f*l1*l4)));
            alpha2 = acos(((l1*l1+l2*l2-l4*l4)/(2.0f*l1*l2)));
            if ((!isnan(gamma))&&(!isnan(alpha1))&&(!isnan(alpha2))) {
                new_pos[0] = gamma + alpha1;
                new_pos[1] = -(3.14159f-alpha2);
                new_pos[2] = link3_corrections[0] - new_pos[0] - new_pos[1]; // + link_angles_des[0]
            }
            if ((new_pos[2]-conv_pos[2])>link3_delta_limit){ // check limits again?
                new_pos[2] = conv_pos[2] + link3_delta_limit;
            } else if ((new_pos[2]-conv_pos[2])<-link3_delta_limit){
                new_pos[2] = conv_pos[2] - link3_delta_limit;
            }
        } else {
            // set nominal pose
            // TODO: change to taking steps towards the nominal pose?
            new_pos[0] = conv_pos[0] + pos_eps*(default_pos[0]-conv_pos[0]);
            new_pos[1] = conv_pos[1] + pos_eps*(default_pos[1]-conv_pos[1]);
            new_pos[2] = link3_corrections[0]-default_pos[0]-default_pos[1];
            if ((new_pos[2]-conv_pos[2])>link3_delta_limit){
                new_pos[2] = conv_pos[2] + link3_delta_limit;
            } else if ((new_pos[2]-conv_pos[2])<-link3_delta_limit){
                new_pos[2] = conv_pos[2] - link3_delta_limit;
            }

        }
        if (right_ik==1){
            // calculate ik based on pstar
            // link3_corrections[1] = link3_corrections[1] - conv_pos[3] - conv_pos[4]; // convert to joint angle to limit movement
            // if ((link3_corrections[1]-conv_pos[5])>link3_delta_limit){
            //     link3_corrections[1] = conv_pos[5] + link3_delta_limit;
            // } else if ((link3_corrections[1]-conv_pos[5])<-link3_delta_limit){
            //     link3_corrections[1] = conv_pos[5] - link3_delta_limit;
            // }
            // link3_corrections[1] = link3_corrections[1] + conv_pos[3] + conv_pos[4]; // convert back to world angle
            x2 = pstar[1][0] - l3*cos(link3_corrections[1]);
            y2 = pstar[1][1] - l3*sin(link3_corrections[1]) - r_yoff;
            l4 = fminf2( sqrt(x2*x2 + y2*y2), l1+l2);
            gamma = atan2(y2,x2);
            alpha1 = acos(((l1*l1+l4*l4-l2*l2)/(2.0f*l1*l4)));
            alpha2 = acos(((l1*l1+l2*l2-l4*l4)/(2.0f*l1*l2)));
            // pc.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n\r", range_m[5], range_m[6], link3_corrections[1], x2, y2, l4, gamma, alpha1, alpha2);
            if ((!isnan(gamma))&&(!isnan(alpha1))&&(!isnan(alpha2))) {
                new_pos[3] = gamma - alpha1;
                new_pos[4] = 3.14159f-alpha2;
                new_pos[5] = link3_corrections[1] - new_pos[3] - new_pos[4]; // + link_angles_des[1]
            } 
            if (new_pos[5]>(conv_pos[5]+link3_delta_limit)){ // check limits again?
                new_pos[5] = conv_pos[5] + link3_delta_limit;
            } else if (new_pos[5]<(conv_pos[5]-link3_delta_limit)){
                new_pos[5] = conv_pos[5] - link3_delta_limit;
            }
        } else {
            // set nominal pose
            // TODO: change to taking steps towards the nominal pose?
            new_pos[3] = conv_pos[3] + pos_eps*(default_pos[3]-conv_pos[3]);
            new_pos[4] = conv_pos[4] + pos_eps*(default_pos[4]-conv_pos[4]);
            new_pos[5] = link3_corrections[1]-default_pos[3]-default_pos[4];
            if (new_pos[5]>(conv_pos[5]+link3_delta_limit)){
                new_pos[5] = conv_pos[5] + link3_delta_limit;
            } else if (new_pos[5]<(conv_pos[5]-link3_delta_limit)){
                new_pos[5] = conv_pos[5] - link3_delta_limit;
            }
        }

        */

        if (left_ik==1){
            des_left[0][0]=400.0f*(pstar[0][0]-p[0][0]); //x position of left x, gain* (desired- actual)
            des_left[1][0]=400.0f*(pstar[0][1]-p[0][1]); //y position of left y, gain* (desired - actual)
            des_left[2][0]=150.0f*(link3_corrections[0]-p[0][2]); //theta of left, gain* (desired- actual)
            mult33x31(tau_left, JT_left, des_left); //get tau left
            //pc.printf("left ik \n\r");
        }
        else {
            tau_left[0][0]=0.0f;
            tau_left[1][0]=0.0f;
            tau_left[2][0]=0.0f;
        }

        if (right_ik==1){
            des_right[0][0]=100.0f*(pstar[1][0]-p[1][0]); //x position of left, gain* (desired- actual)
            des_right[1][0]=100.0f*(pstar[1][1]-p[1][1]); //y position of left, gain* (desired - actual)
            des_right[2][0]=100.0f*(link3_corrections[1]-p[1][2]); //theta of left, gain* (desired- actual)
            mult33x31(tau_right, JT_right, des_right); //get tau left
        }
        else {
            tau_right[0][0]=0.0f;
            tau_right[1][0]=0.0f;
            tau_right[2][0]=0.0f;
        }
        
        des_cur[0]=(int16_t)tau_left[0][0];
        des_cur[1]=(int16_t)tau_left[1][0];
        des_cur[2]=(int16_t)tau_left[2][0];
        des_cur[3]=(int16_t)tau_right[0][0];
        des_cur[4]=(int16_t)tau_right[1][0];
        des_cur[5]=(int16_t)tau_right[2][0];


        /*
        // set new desired joint positions (in counts)
        for (int i=0; i<num_IDs; i++){
            //des_pos[i] = (uint32_t)((new_pos[i]+dxl_offsets[i])/pulse_to_rad); // use ik positions
            des_cur[i] = (uint16_t);
            // des_pos[i] = (uint32_t)((default_pos[i]+dxl_offsets[i])/pulse_to_rad); // just hold default positions
        }
        */

        dxl_bus.SetMultGoalCurrents(dxl_IDs, num_IDs, des_cur);

        // // set link 3 angles correctly, checking that the change in angle isn't too large
        // new_pos[2] = link3_corrections[0]-default_pos[0]-default_pos[1];
        // if ((new_pos[2]-conv_pos[2])>link3_delta_limit){
        //     new_pos[2] = conv_pos[2] + link3_delta_limit;
        // } else if ((new_pos[2]-conv_pos[2])<-link3_delta_limit){
        //     new_pos[2] = conv_pos[2] - link3_delta_limit;
        // }
        // new_pos[5] = link3_corrections[1]-default_pos[3]-default_pos[4];
        // if (new_pos[5]>(conv_pos[5]+link3_delta_limit)){
        //     new_pos[5] = conv_pos[5] + link3_delta_limit;
        // } else if (new_pos[5]<(conv_pos[5]-link3_delta_limit)){
        //     new_pos[5] = conv_pos[5] - link3_delta_limit;
        // }
        // des_pos[2] = (uint32_t)((new_pos[2]+dxl_offsets[2])/pulse_to_rad);
        // des_pos[5] = (uint32_t)((new_pos[5]+dxl_offsets[5])/pulse_to_rad);

        //dxl_bus.SetMultGoalPositions(dxl_IDs, num_IDs, des_pos);

        // loop takes about 5.5ms without printing

        t2.reset();
         /*
        pc.printf("%.2f, %d, %d, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, ", t3.read(), samp1, samp2, range_m[0], range_m[1], range_m[2], range_m[3], range_m[4], range_m[5], range_m[6], range_m[7], range_m[8]);
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f, %2.3f, ", conv_pos[0], conv_pos[1], conv_pos[2], conv_pos[3], conv_pos[4], conv_pos[5]);
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f, %2.3f, ", p[0][0], p[0][1], p[0][2], p[1][0], p[1][1], p[1][2]); // forward kinematics
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f, %2.3f\n\r", new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], new_pos[5]); // new positions in radians
        */
        pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f \n\r", range_m[2], range_m[3], tau_left[0][0], tau_left[1][0], tau_left[2][0]); 
        // printing data takes about 4ms at baud of 460800
        // pc.printf("%.2f, %d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", t3.read(), range[0], range[1], range[2], range[3], range[4], range[5], range[6], range[7], range[8]);
        // pc.printf("%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f\n\r", l_diff, l_avg, r_diff, r_avg, link3_corrections[0], link3_corrections[1]);
        
        // pc.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", range_status[0], range_status[1], range_status[2], range_status[3], range_status[4], range_status[5], range_status[6], range_status[7], range_status[8]);  
        // pc.printf("%d, %d, %d, %d, %d, %d\n\r", dxl_pos[0], dxl_pos[1], dxl_pos[2], dxl_pos[3], dxl_pos[4], dxl_pos[5]);

        // pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f, ", left_finger.output_data[0], left_finger.output_data[1], left_finger.output_data[2], left_finger.output_data[3], left_finger.output_data[4]);
        // pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f\n\r", right_finger.output_data[0], right_finger.output_data[1], right_finger.output_data[2], right_finger.output_data[3], right_finger.output_data[4]);
        // print desired joint positions and forward kinematics instead of force sensor values

        // TODO: only print every five or ten loops?
        // pc.printf("%d, %d\n\r\n\r", samp4, samp5);

        samp4 = t2.read_us();
        samp5 = t.read_us();
        wait_us(10);

        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}