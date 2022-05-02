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

Serial pc(USBTX, USBRX, 921600); //460800);
DigitalOut led(LED1);

float loop_time = 0.01f; // 200hz is probably maximum sample rate since that is pressure sensor rate too
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
float p2[2][3]; //new variable of only the first 2 link
float v[2][3];
float etip_left[3][2]; // out, fw, in
float etip_right[3][2]; // out, fw, in
float J_left[2][2]; // 
float JT_left[2][2]; // 
float tau_left[3][1];// 
float des_left[3][1];// 
float J_right[2][2]; // 
float JT_right[2][2]; // 
float tau_right[3][1];// 
float des_right[3][1];// 
float pstar[2][2]; // desired change in end-effector position
float prox_thresh = 0.030f; // avoidance threshold for older avoiding using outer sensor experiment 
float glide_thresh = 0.120f; // glide threshold
float corr_thresh = 0.080f; // link 3 correction threshold
float avoid_thresh_f = 0.070f; //avoidance threshold of forward
float avoid_thresh_o = 0.060f; //avoidance threshold ofr both
float ang_thresh = 0.0f; // angle threshold, in mm difference 
float ang_max = 1.5; // maximum angle correction from nominal
float ang_min = -1.5; // minimum angle correction from nominal
float glide_dist = 0.050f; // desired contact distance for gliding/contour following
float link3_delta_limit = 0.2f; // maximum change in position command
int left_ik = 0;
int right_ik = 0;
float link3_corrections[2]; // corrections to link 3 angles based on difference in proximity measurements
float x2, y2, l4, gamma, alpha1, alpha2;  
float pos_eps = 0.3; // exponential return to the default pose
float l_diff, l_avg, r_diff, r_avg; // values for internal sensors
float Kd; //joint damping
float Kp1; //proportaional gain for x y
float Kp2; //proportaional gain for angle

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

    
    pc.printf("Initializing Dynamixels.\n\r");
    // Enable dynamixels and set control mode...individual version
    for(int i=0; i<6; i++){
        pc.printf("Motor ID %d.\n\r",dxl_IDs[i]);
        dxl_bus.SetTorqueEn(dxl_IDs[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_IDs[i],0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_IDs[i], POSITION_CONTROL);
        
        dxl_bus.SetTorqueEn(dxl_IDs[i], 0x01); //to be able to move 
        des_pos[i] = (uint32_t)((default_pos[i]+dxl_offsets[i])/pulse_to_rad);
        dxl_bus.SetGoalPosition(dxl_IDs[i], des_pos[i]);
        wait_us(100);
    } 

    for(int i=0; i<6; i++){
        pc.printf("Motor ID %d.\n\r",dxl_IDs[i]);
        dxl_bus.SetTorqueEn(dxl_IDs[i],0x00);    
        dxl_bus.SetRetDelTime(dxl_IDs[i],0x32); // 4us delay time?
        dxl_bus.SetControlMode(dxl_IDs[i], CURRENT_CONTROL);
        dxl_bus.SetTorqueEn(dxl_IDs[i], 0x01); //to be able to move 
        wait_us(100);
    } 

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
        dxl_bus.GetMultPositions(dxl_pos, dxl_IDs, num_IDs);
        // convert states
        for(int i=0; i<num_IDs; i++){
            conv_pos[i] = (pulse_to_rad*(float)dxl_pos[i])-dxl_offsets[i];
        }
        samp3 = t2.read_us();

        //new
        // evaluate forward kinematics of p2 (only 1st 2 link) and p (final link)
        p2[0][0] = l1*cos(conv_pos[0]) + l2*cos(conv_pos[0]+conv_pos[1]) ; // left x
        p2[0][1] = l_yoff + l1*sin(conv_pos[0]) + l2*sin(conv_pos[0]+conv_pos[1]) ; // left y 
        p2[0][2] = conv_pos[0]+conv_pos[1]; // left theta
        p[0][0] = p2[0][0] + l3*cos(conv_pos[0]+conv_pos[1]+conv_pos[2]); // left x 
        p[0][1] = p2[0][1]+ l3*sin(conv_pos[0]+conv_pos[1]+conv_pos[2]); // left y // technically b
        p[0][2] = p2[0][2] + conv_pos[2]; // left theta distal in world frame

        etip_left[0][0] = -sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[0][1] = cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[1][0] = cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[1][1] = sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[2][0] = sin(conv_pos[0]+conv_pos[1]+conv_pos[2]);
        etip_left[2][1] = -cos(conv_pos[0]+conv_pos[1]+conv_pos[2]);

        //Jacobian for Left P2 not p
        J_left[0][0] = -l1*sin(conv_pos[0]) -l2*sin(conv_pos[0]+conv_pos[1]);// dq1/dx with q1=conv_pos[0] or base
        J_left[0][1] = -l2*sin(conv_pos[0]+conv_pos[1]);// dq2/dx with q2=conv_pos[1]
        J_left[1][0] =  l1*cos(conv_pos[0]) + l2*cos(conv_pos[0]+conv_pos[1]); // dq1/dy
        J_left[1][1] =  l2*cos(conv_pos[0]+conv_pos[1]); // dq2/dy

        // evaluate forward kinematics
        p2[1][0] = l1*cos(conv_pos[3]) + l2*cos(conv_pos[3]+conv_pos[4]); //x
        p2[1][1] = r_yoff + l1*sin(conv_pos[3]) + l2*sin(conv_pos[3]+conv_pos[4]); //y
        p2[1][2] = conv_pos[3]+conv_pos[4];
        p[1][0] = p2[1][0] + l3*cos(conv_pos[3]+conv_pos[4]+conv_pos[5]); // right x
        p[1][1] = p2[1][1] + l3*sin(conv_pos[3]+conv_pos[4]+conv_pos[5]); // right y *technically off by 9mm to the center of sphere
        p[1][2] = p2[1][2] + conv_pos[5]; // right theta

        etip_right[0][0] = sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[0][1] = -cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[1][0] = cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[1][1] = sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[2][0] = -sin(conv_pos[3]+conv_pos[4]+conv_pos[5]);
        etip_right[2][1] = cos(conv_pos[3]+conv_pos[4]+conv_pos[5]);

        //Jacobian for right REWRITE
        J_right[0][0] = -l1*sin(conv_pos[3]) -l2*sin(conv_pos[3]+conv_pos[4]);// dq1/dx with q1=conv_pos[0] or base
        J_right[0][1] = -l2*sin(conv_pos[3]+conv_pos[4]);// dq2/dx with q2=conv_pos[1]
        J_right[1][0] =  l1*cos(conv_pos[3]) + l2*cos(conv_pos[3]+conv_pos[4]); // dq1/dy
        J_right[1][1] =  l2*cos(conv_pos[3]+conv_pos[4]) ; // dq2/dy

        //Jacobian transpose for both left and right   
        for (int i=0; i<2; i++){
            for (int j=0; j<2; j++){
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

        // Calculate "gliding" distance
        l_diff = 0.0f;
        l_avg = 0.0f;
        r_diff = 0.0f;
        r_avg = 0.0f;

        Kp1= 10000*3; //with 60000 0 0 springy
        Kp2= 50*3;
        Kd = 0;//0.04;
        
        tau_left[0][0]=0.0f;
        tau_left[1][0]=0.0f;
        tau_left[2][0]=0.0f;
        tau_right[0][0]=0.0f;
        tau_right[1][0]=0.0f;
        tau_right[2][0]=0.0f;

        if ((range_m[2]<glide_thresh)&&(range_m[3]<glide_thresh)){ // if both sensors are activated for gliding 
            l_diff = fmaxf2( fminf2( range_m[3]-range_m[2], corr_thresh), -corr_thresh);
            link3_corrections[0] = atan2(l_diff,linbw);

            if (((link3_corrections[0] + conv_pos[2]) <ang_max) && ((link3_corrections[0] + conv_pos[2]) >ang_min)){ //checking if its past limit
                link3_corrections[0] = fmaxf2( fminf2( link3_corrections[0], ang_max), ang_min) + p[0][2]; //the angle of the normal in world frame by adding p[0][2]= angle of 3rd link
                left_ik = 1;
                l_avg = 0.5*(range_m_raw[2]+range_m_raw[3]);
                pstar[0][0] += l_avg*etip_left[2][0]; // x: get from p to surface
                pstar[0][0] += -glide_dist*sin(link3_corrections[0]); // x: from surface to p of desired 
                pstar[0][0] += -l3*cos(link3_corrections[0]); // x: from p of desired to p2 desired
            
                pstar[0][1] += l_avg*etip_left[2][1]; // y: get from p to surface
                pstar[0][1] += -glide_dist*-cos(link3_corrections[0]);// y: from surface to p of desired 
                pstar[0][1] += -l3*sin(link3_corrections[0]);// y: from p of desired to p2 desired
                
                des_left[0][0]=Kp1*(pstar[0][0]-p2[0][0]); //x position of left x, gain* (desired- actual)
                des_left[1][0]=Kp1*(pstar[0][1]-p2[0][1]); //y position of left y, gain* (desired - actual)
                tau_left[0][0] = JT_left[0][0]*des_left[0][0] + JT_left[0][1]*des_left[1][0] ;
                tau_left[1][0] = JT_left[1][0]*des_left[0][0] + JT_left[1][1]*des_left[1][0] ;
                tau_left[2][0] =  Kp2 * (link3_corrections[0]-p[0][2]);
                //pc.printf("left ik \n\r");
            } else{
                pc.printf("Left Joint limits \n\r");
            }

        }

        

        if ((range_m[5]<glide_thresh)&&(range_m[6]<glide_thresh)){ 
            l_diff = fmaxf2( fminf2( range_m[6]-range_m[5], corr_thresh), -corr_thresh);
            link3_corrections[1] = atan2(l_diff,linbw);

            if (((link3_corrections[1] + conv_pos[5]) <ang_max) && ((link3_corrections[1] + conv_pos[5]) >ang_min)){ 
                link3_corrections[1] = fmaxf2( fminf2( link3_corrections[1], ang_max), ang_min) + p[1][2]; 
                right_ik = 1;
                r_avg = 0.5*(range_m_raw[5]+range_m_raw[6]);
                pstar[1][0] += r_avg*etip_right[2][0];
                pstar[1][0] += -glide_dist*-sin(link3_corrections[1]);
                pstar[1][0] += -l3*cos(link3_corrections[1]); 
            
                pstar[1][1] += r_avg*etip_right[2][1]; 
                pstar[1][1] += -glide_dist*cos(link3_corrections[1]);
                pstar[1][1] += -l3*sin(link3_corrections[1]);

                des_right[0][0]= 1* Kp1*(pstar[1][0]-p2[1][0]); //x position of left x, gain* (desired- actual)
                des_right[1][0]= 1* Kp1*(pstar[1][1]-p2[1][1]); //y position of left y, gain* (desired - actual)
                tau_right[0][0] = JT_right[0][0]*des_right[0][0] + JT_right[0][1]*des_right[1][0] ;
                tau_right[1][0] = JT_right[1][0]*des_right[0][0] + JT_right[1][1]*des_right[1][0] ;
                tau_right[2][0] =  1.5* Kp2 * (link3_corrections[1]-p[1][2]);
            //pc.printf("left ik \n\r");
            } else{
                pc.printf("Right joint limits \n\r");
            }

        }

        if ((range_m[0]<avoid_thresh_o)||(range_m[1]<avoid_thresh_f)){ // LEFT SENSOR: if either back sensor less than avoidance

            des_left[0][0] = Kp1*2*(fmaxf2(avoid_thresh_f-range_m[1], 0.0f)* -cos(p[0][2])) + Kp1*1*(fmaxf2(avoid_thresh_o-range_m[0], 0.0f)* sin(p[0][2]));//x position CHANGE THIS TO ANOTHER VARIABLE
            des_left[1][0] = Kp1*2*(fmaxf2(avoid_thresh_f-range_m[1], 0.0f)* -sin(p[0][2])) + Kp1*1*(fmaxf2(avoid_thresh_o-range_m[0], 0.0f)* -cos(p[0][2]));//y position 
            tau_left[0][0] += JT_left[0][0]*des_left[0][0] + JT_left[0][1]*des_left[1][0] ;
            tau_left[1][0] += JT_left[1][0]*des_left[0][0] + JT_left[1][1]*des_left[1][0] ;

        }
        if ((range_m[8]<avoid_thresh_o)||(range_m[7]<avoid_thresh_f)){ // RIGHT SENSORi: f either back sensor less than avoidance

            des_right[0][0] = Kp1*1*(fmaxf2(avoid_thresh_f-range_m[7], 0.0f)* -cos(p[1][2])) + Kp1*(fmaxf2(avoid_thresh_o-range_m[8], 0.0f)* -sin(p[1][2]));//x position CHANGE THIS TO ANOTHER VARIABLE
            des_right[1][0] = Kp1*1*(fmaxf2(avoid_thresh_f-range_m[7], 0.0f)* -sin(p[1][2])) + Kp1*(fmaxf2(avoid_thresh_o-range_m[8], 0.0f)* cos(p[1][2]));//y position 
            tau_right[0][0] += JT_right[0][0]*des_right[0][0] + JT_right[0][1]*des_right[1][0] ;
            tau_right[1][0] += JT_right[1][0]*des_right[0][0] + JT_right[1][1]*des_right[1][0] ;

        }


        des_cur[0]=(int16_t)tau_left[0][0];
        des_cur[1]=(int16_t)tau_left[1][0];
        des_cur[2]=(int16_t)tau_left[2][0];
        des_cur[3]=(int16_t)tau_right[0][0];
        des_cur[4]=(int16_t)tau_right[1][0];
        des_cur[5]=(int16_t)tau_right[2][0];


        dxl_bus.SetMultGoalCurrents(dxl_IDs, num_IDs, des_cur);

    

        t2.reset();
        
        pc.printf("%.2f, %1.3f, %1.3f,    %1.4f, %1.4f,    %1.3f,    %1.4f, %1.4f,   %1.3f, %1.3f\n\r", t3.read(), range_m[0], range_m[1], range_m[2], range_m[3], range_m[4], range_m[5], range_m[6], range_m[7], range_m[8]);
        //pc.printf("%2.3f, %2.3f, %2.3f, %2.3f, %2.3f, %2.3f\n\r", conv_pos[0], conv_pos[1], conv_pos[2], conv_pos[3], conv_pos[4], conv_pos[5]);
        //pc.printf("%2.3f, %.2f,%2.3f, %2.3f, %2.3f,%2.3f,%2.3f, %2.3f,%2.3f,%2.3f, %d, %d, %d \n\r", t3.read(), range_m[2], range_m[3], p[0][0], des_left[0][0],des_left[1][0],des_left[2][0], tau_left[0][0], tau_left[1][0], tau_left[2][0], des_cur[0], des_cur[1], des_cur[2]); //left side
//        pc.printf("%2.3f, %2.3f,%2.3f, %2.3f, %2.3f,%2.3f,%2.3f, %2.3f,%2.3f,%2.3f, %d, %d, %d \n\r", t3.read(), range_m[5], range_m[6], p[0][0], des_right[0][0],des_right[1][0],des_right[2][0], tau_right[0][0], tau_right[1][0], tau_right[2][0], des_cur[3], des_cur[4], des_cur[5]); //right side                pc.printf("%2.3f, %.2f,%2.3f, %2.3f, %2.3f,%2.3f,%2.3f, %2.3f,%2.3f,%2.3f, %d, %d, %d \n\r", t3.read(), range_m[5], range_m[6], p[0][0], des_right[0][0],des_right[1][0],des_right[2][0], tau_right[0][0], tau_right[1][0], tau_right[2][0], des_cur[3], des_cur[4], des_cur[5]);
//        pc.printf("%2.3f, %2.3f,%2.3f,  %2.3f,   %2.3f,%2.3f,%2.3f, %2.3f,%2.3f,%2.3f, %d, %d, %d \n\r", t3.read(), range_m[1], range_m[0], p[0][2], des_left[0][0],des_left[1][0],des_left[2][0], tau_left[0][0], tau_left[1][0], tau_left[2][0], des_cur[0], des_cur[1], des_cur[2]); // right sensor  
        //pc.printf("%2.3f, %2.3f,%2.3f,  %2.3f,   %2.3f,%2.3f,%2.3f, %2.3f,%2.3f,%2.3f, %d, %d, %d \n\r", t3.read(), range_m[8], range_m[7], p[1][2], des_right[0][0],des_right[1][0],des_right[2][0], tau_right[0][0], tau_right[1][0], tau_right[2][0], des_cur[3], des_cur[4], des_cur[5]); //


        samp4 = t2.read_us();
        samp5 = t.read_us();
        wait_us(10);

        while (t.read()<loop_time) {;}
        led = !led;
        
    }
}