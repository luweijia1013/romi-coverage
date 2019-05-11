#ifndef _IRProximity_h
#define _IRProximity_h

#include <Arduino.h>

class SharpIR
{
public:
    SharpIR(byte pin, float _param1, float _param2, float _param3, float _param4);
    int  getRaw();
    byte  getRaw2();
    float  getDistanceInMM();
    float  getDistance();
    void   calibrate();
    float  read_calibrated();
    void   findChangeSpeed();
    float  lowpass_filter();
    byte  myFilter();

    float  y;
    float  alpha = 0.500;
    float  const anlg2volt = 0.0048828125; //(5/1024)
    float  oldDistance=0;
    float  changeSpeed=0;

private:
    byte pin;
    float param1;
    float param2;
    float param3;
    float param4;
};

SharpIR::SharpIR(byte _pin, float _param1, float _param2, float _param3, float _param4)
{
    pin = _pin;
    param1 = _param1;
    param2 = _param2;
    param3 = _param3;
    param4 = 1/_param4;
}

int SharpIR::getRaw()
{
    return analogRead(pin);
}

byte SharpIR::getRaw2()
{
    return constrain(analogRead(pin)/3, 0, 255);
}

float SharpIR::read_calibrated()
{
    //return lowpass_filter()*anlg2volt;
    return getRaw()*anlg2volt;
}

float SharpIR::lowpass_filter()
{
    y = alpha*y + (1-alpha) * getRaw();
    return y;
}

byte SharpIR::myFilter()
{
    const byte size = 30;
    byte raw[size] = {0};
    for (byte i = 0; i < size; i++) {
        raw[i] = getRaw2();
    }

    //printMassive(raw, size);
    return BoyerMooreMajority(raw, size);
}

/*
 * This piece of code is quite crucial to mapping
 * obstacle distance accurately, so you are encouraged
 * to calibrate your own sensor by following the labsheet.
 * Also remember to make sure your sensor is fixed to your
 * Romi firmly and has a clear line of sight!
 */
float SharpIR::getDistanceInMM()
{

    //float distance = (float)analogRead( pin );

    // map this to 0 : 5v range.
    //distance *= 0.0048;

    const float exponent = (1/param2);
    float distance = pow( ( read_calibrated() / param1 ), exponent);
    return distance;
}

void SharpIR::findChangeSpeed()
{
  float newDistance = getDistanceInMM();
  changeSpeed = abs(newDistance-oldDistance);
  oldDistance = newDistance;
}

float SharpIR::getDistance()
{
    return pow( ( (float) myFilter() / param3 ), param4);
}


#endif


