#ifndef MAIN_H
#define MAIN_H

#define S0           9
#define S1           8
#define S2           4
#define S3           5

#define LDRPINCOUNT 15

void selectMUXChannel(uint8_t channel);
int readMUXChannel(int index);

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

    double maxRecordedValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double minRecordedValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double calculatedThresholdValue[LDRPINCOUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double highValues[LDRPINCOUNT] = {3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30,  
                                      3.30, 3.30, 3.30, 3.30, 3.30, 3.30, 3.30};
};

#endif