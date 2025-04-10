#include "util.h"

#include <Arduino.h>
#include <Wire.h>

// clips angle between range of (-180, 180)
double clipAngleto180degrees(double angle) {
    if (angle < 0) angle = 360 + angle;
    angle = fmod(angle, 360);
    return (angle > 180) ? angle - 360 : angle;
}

double clipAngleto360degrees(double angle) {
    angle = fmod(angle, 360);
    return angle < 0 ? angle + 360 : angle;
}

// Trigonometric functions that work in degrees clipped from (-180, 180]

double atand(double y, double x) {
    return clipAngleto180degrees(atan2(y, x) * RAD_TO_DEG);
}

void printDouble(Stream &serial, double value, uint8_t integerPlaces,
                 uint8_t decimalPlaces) {
    const auto integerComponent = (int)value;
    const auto decimalComponent =
        (int)round(abs(value * pow(10, decimalPlaces))) %
        (int)pow(10, decimalPlaces);

    if (integerPlaces == 0)
        serial.printf("%d", integerComponent);
    else
        serial.printf("%*d", integerPlaces, integerComponent);
    serial.print(".");
    if (decimalPlaces == 0)
        serial.printf("%d", decimalComponent);
    else
        serial.printf("%0*d", decimalPlaces, decimalComponent);
}

double sind(double angle) { return sin(angle * DEG_TO_RAD); }
double cosd(double angle) { return cos(angle * DEG_TO_RAD); }

#define WIRE Wire

void i2cscanner(){
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        WIRE.beginTransmission(address);
        error = WIRE.endTransmission();

        if (error == 0)
        {
        Serial.print("I2C device found at address 0x");
        if (address<16)
            Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");

        nDevices++;
        }
        else if (error==4)
        {
        Serial.print("Unknown error at address 0x");
        if (address<16)
            Serial.print("0");
        Serial.println(address,HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(1000);           // wait 1 seconds for next scan
}

