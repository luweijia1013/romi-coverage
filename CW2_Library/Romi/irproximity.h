#ifndef _IRProximity_h
#define _IRProximity_h

class SharpIR
{
    public:
        SharpIR(byte pin);
        int  getDistanceRaw();
        float getDistanceInMM();
        float getVoltage();
        void calibrate();

    private:
        byte pin;
};

SharpIR::SharpIR(byte _pin)
{
  pin = _pin;
}

int SharpIR::getDistanceRaw()
{
    return analogRead(pin);
}

float SharpIR::getVoltage(){
    return analogRead(pin) * 5 / 1023.0;
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
    
    float distance = (float)analogRead( pin );
    
    // map this to 0 : 5v range.
    distance = distance * 5 / 1023.0;

    const float exponent = (1/-0.691);
    distance = pow( ( distance / 15.661 ), exponent);
       
    return distance * 10;
}


#endif
