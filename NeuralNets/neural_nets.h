#ifndef NEURAL_NETS_H
#define NEURAL_NETS_H

// define struct for neural nets here
struct NeuralNetOLD{
    
    float b1[12];
    float b2[25];
    float b3[5];
    
    float w1[8][12];
    float w2[12][25];
    float w3[25][5];
    
    float minims[13];
    float maxims[13];
    float max_pressure;
    
    };
    
struct NeuralNet{
    
    float b1[12];
    float b2[64];
    float b3[64];
    float b4[5];
    
    float w1[8][12];
    float w2[12][64];
    float w3[64][64];
    float w4[64][5];
    
    float minims[13];
    float maxims[13];
    //float max_pressure;
    
    };

#endif



