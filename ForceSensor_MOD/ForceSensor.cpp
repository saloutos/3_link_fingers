
#include "mbed.h"
#include "ForceSensor.h"
#include "../math_ops.h"
//#include <math.h>

ForceSensor::ForceSensor(int channel_number, NeuralNet *neural_net){
    
    // store channel number and pointer for neural net struct
    // TODO: implement check that channel is 1, 2, or 3
    _channel = channel_number;
    _net = neural_net;
    
    // sensors are structs are already initialized
    
    // initialize other values
    sensor_comp = uint8_t(1)| uint8_t(1<<1); // sensor_comp = BMP3_PRESS | BMP3_TEMP;
    
    }
    
void ForceSensor::Initialize(){
    
    // configure sensor devices
    if (_channel==1){    
        writeHigh1(1);
        writeHigh1(2);
        writeHigh1(3);
        writeHigh1(4);
        writeHigh1(5);
        writeHigh1(6);
        writeHigh1(7);
        writeHigh1(8);
    } else if (_channel==2) {
        writeHigh2(1);
        writeHigh2(2);
        writeHigh2(3);
        writeHigh2(4);
        writeHigh2(5);
        writeHigh2(6);
        writeHigh2(7);
        writeHigh2(8);
    }
    
    printf("Initializing force sensor.\n\r");

    s1.dev_id = 1;  // tells which cs pin associated with device
    config_dev(&s1, _channel);
    s2.dev_id = 2;  // tells which cs pin associated with device
    config_dev(&s2, _channel);
    s3.dev_id = 3;  // tells which cs pin associated with device
    config_dev(&s3, _channel);
    s4.dev_id = 4;  // tells which cs pin associated with device
    config_dev(&s4, _channel);
    s5.dev_id = 5;  // tells which cs pin associated with device
    config_dev(&s5, _channel);
    s6.dev_id = 6;  // tells which cs pin associated with device
    config_dev(&s6, _channel);
    s7.dev_id = 7;  // tells which cs pin associated with device
    config_dev(&s7, _channel);
    s8.dev_id = 8;  // tells which cs pin associated with device
    config_dev(&s8, _channel);
    
}
    
    
void ForceSensor::config_dev(struct bmp3_dev *dev, int channel){
    int8_t rslt=0;//BMP3_OK; // get error with rslt = BMP3_OK;
    
    dev -> intf = BMP3_SPI_INTF;


    if (channel==1){
        dev -> read = &bmp_spi1_read;
        dev -> write = &bmp_spi1_write;
    } else if (channel==2) {
        dev -> read = &bmp_spi2_read;
        dev -> write = &bmp_spi2_write;
    }
    
    dev -> delay_ms = &bmp_delay_ms;
    rslt = bmp3_init(dev);
    printf("* initialize sensor result = 0x%x *\r\n", rslt);
    wait(0.25);
    
    // ***** Configuring settings of sensor
    // Normal Mode - bmp3_set_op_mode
    // Temp En, Press En 
    // OSR = no oversampling temp, press
    // ODR = 200Hz temp, press
    // IRR = no IRR filter
    // ^^^all 4 above =  bmp3_set_sensor_settings
    
    // Set sensor settings (press en, temp en, OSR, ODR, IRR)
    dev -> settings.press_en = 0x01; // BMP3_ENABLE
    dev -> settings.temp_en = 0x01; //BMP3_ENABLE
    dev -> settings.odr_filter.press_os = 0x00; //BMP3_NO_OVERSAMPLING
    dev -> settings.odr_filter.temp_os = 0x00; //BMP3_NO_OVERSAMPLING
    dev -> settings.odr_filter.odr = 0x00; //BMP3_ODR_200_HZ
    dev -> settings.odr_filter.iir_filter = 0x00; //BMP3_IIR_Filter_disable
    
    uint16_t settings_sel;
    //settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_IIR_FILTER_SEL | BMP3_ODR_SEL;
    settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2) | uint16_t(1 << 4) | uint16_t(1 << 5) | uint16_t(1 << 6) | uint16_t(1 << 7);
    //settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2);
    rslt = bmp3_set_sensor_settings(settings_sel, dev);
    
    // Set operating (power) mode
    dev -> settings.op_mode = 0x03; /// normal mode = 0x03
    rslt = bmp3_set_op_mode(dev);
    
    // Check settings
    rslt = bmp3_get_sensor_settings(dev);  
}
    
void ForceSensor::Sample(){
    
    // get data from every sensor
    bmp3_get_sensor_data(sensor_comp, &data1, &s1);        
    bmp3_get_sensor_data(sensor_comp, &data2, &s2);
    bmp3_get_sensor_data(sensor_comp, &data3, &s3);
    bmp3_get_sensor_data(sensor_comp, &data4, &s4);
    bmp3_get_sensor_data(sensor_comp, &data5, &s5);
    bmp3_get_sensor_data(sensor_comp, &data6, &s6);
    bmp3_get_sensor_data(sensor_comp, &data7, &s7);
    bmp3_get_sensor_data(sensor_comp, &data8, &s8);
    
    // store data
    raw_data[0] = int(data1.pressure)-100000; // pressure is returned in Pa, could subtract actual sea level pressure here
    raw_data[1] = int(data2.pressure)-100000;
    raw_data[2] = int(data3.pressure)-100000;
    raw_data[3] = int(data4.pressure)-100000;
    raw_data[4] = int(data5.pressure)-100000;
    raw_data[5] = int(data6.pressure)-100000;
    raw_data[6] = int(data7.pressure)-100000;
    raw_data[7] = int(data8.pressure)-100000;
    
    // could combine this with previous step
    offset_data[0] = raw_data[0]-offsets[0];
    offset_data[1] = raw_data[1]-offsets[1];
    offset_data[2] = raw_data[2]-offsets[2];
    offset_data[3] = raw_data[3]-offsets[3];
    offset_data[4] = raw_data[4]-offsets[4];
    offset_data[5] = raw_data[5]-offsets[5];
    offset_data[6] = raw_data[6]-offsets[6];
    offset_data[7] = raw_data[7]-offsets[7];
    
}
    
void ForceSensor::Evaluate(){
    // scales raw input data, evaluates neural network, scales and stores output data
        
    // scale sensor data
    for (int i=0; i<8; i++){
        input_data[i] = 0.0f;
        input_data[i] = (((float)offset_data[i]) - (_net->minims[i+5]))/(_net->maxims[i+5]-_net->minims[i+5]); // / _net->max_pressure;
        // check that inputs are between 0 and 1?
    
    }

    // decode sensor data here....521*4 operations (multiply,add,activation,add)
    // reset values
    for (int i = 0; i<12; i++){
        l1[i] = 0.0f;
    }
    for (int i = 0; i<64; i++){ //i<25
        l2[i] = 0.0f;
        l3[i] = 0.0f;
    }
    for (int i = 0; i<5; i++){
        l4[i] = 0.0f;
    }
        
    // layer 1
    for(int i = 0; i<12; i++){ // for each node in the next layer
        for(int j = 0; j<8; j++){ // add contribution of node in prev. layer
            l1[i] +=  (_net->w1[j][i]*input_data[j]); 
        }
        l1[i] += _net->b1[i]; // add bias
        l1[i] = fmaxf(0.0f, l1[i]); // relu activation
    }
        
    // layer 2
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<12; j++){ // add contribution of node in prev. layer
            l2[i] += (_net->w2[j][i]*l1[j]);
        }
        l2[i] += _net->b2[i]; // add bias
        l2[i] = fmaxf(0.0f, l2[i]); // relu activation
    }   
    
    // layer 3 // added for larger network architecture
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l3[i] += (_net->w3[j][i]*l2[j]);
        }
        l3[i] += _net->b3[i]; // add bias
        l3[i] = fmaxf(0.0f, l3[i]); // relu activation
    }   
    
    // layer 4
    for(int i = 0; i<5; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l4[i] += _net->w4[j][i]*l3[j]; 
        }
        l4[i] += _net->b4[i];// add bias
        l4[i] = fmaxf(0.0f, l4[i]); // relu activation
    }  

    // post-process, re-scale decoded data
    for (int i=0; i<5; i++) {
        output_data[i] = 0.0f;
        output_data[i] = (l4[i]*(_net->maxims[i]-_net->minims[i])) + _net->minims[i]; // - abs(_net->minims[i]);
    
    }      
    
}
    
void ForceSensor::Calibrate(){
    
    printf("Calculating sensor offsets.\r\n");
    int samps = 500;
    float temp_offsets[8];
    for (int i=0; i<samps; i++){
        Sample();
        for (int j=0; j<8; j++){
            temp_offsets[j] += ((float)raw_data[j])/((float)samps);
        }
        wait_us(5000); // wait for 1/200 s
    }
    for (int i=0; i<8; i++){
        offsets[i] = (int)temp_offsets[i];
    }

} 
    
    




    





//void calibrateSensor(uint16_t* offsets){
//    
//    float temp_offsets[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//    int samps = 10;
//    
//    for (int i=0; i<samps; i++){
//        for (int j=0; j<8; j++){
//            temp_offsets[j] += (float)spi3.binary(j);
//        }
//        wait_ms(1);
//    }
//    
//    for (int i=0; i<8; i++){
//        temp_offsets[i] = temp_offsets[i]/((float)samps); // get overall offset
//        offsets[i] = (uint16_t)temp_offsets[i]; // convert to int
//    }
//
//    }