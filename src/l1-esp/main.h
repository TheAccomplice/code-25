#ifndef MAIN_H
#define MAIN_H
#include "util.h"

#define S0           D9
#define S1           D8
#define S2           D4
#define S3           D5
#define M1           A0

#define LDRPINCOUNT 15

void selectMUXChannel(uint8_t channel);
int readMUXChannel(int index);
void getValues();
void findLine();


struct LightArray {
    float RAWLDRVALUES[LDRPINCOUNT] = {
        5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00,
        5.00, 5.00, 5.00};
    
    int LDRPINMap[LDRPINCOUNT]{0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                               12, 13, 14}; //starting from back

    float LDRBearings[LDRPINCOUNT]{
        180.0, 204.0, 228.0, 252.0, 276.0, 300.0, 324.0, 348.0, 12.0, 36.0, 
        60.0, 84.0, 108.0, 132.0, 156.0}; // assuming placement of ldrs is constant
    
    float LDRThresholds[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    
                                        //assumes threshold is measured

    double maxRecordedValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double minRecordedValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double calculatedThresholdValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double highValues[LDRPINCOUNT] = {3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30,  
                                      3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30};
};

//global variables 
extern LightArray lightArray;
extern SensorValues1 sensorValues1;

#endif