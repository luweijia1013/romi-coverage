#include <WString.h>
#include <USBAPI.h>
#include "line_sensors.h"

extern LineSensor    LineLeft;
extern LineSensor    LineCentre;
extern LineSensor    LineRight;
extern float left_speed_demand;
extern float right_speed_demand;
extern bool use_speed_controller;
extern int mode;
int linestate = 0;


float rad2deg(float rad)
{
    return rad * (180.0 / PI);
}

float deg2rad(float deg)
{
    return deg * (PI/ 180.0);
}


/*
 *  This is quite a computationally expensive routine,
 *  so you might want to consider not using it.  But
 *  gaussian random numbers are really nice for a random
 *  walk behaviour :)
 *  From: http://www.taygeta.com/random/gaussian.html
 */
float randGaussian( float mean, float sd ) {
    float x1, x2, w, y;

    do {
        // Adaptation here because arduino random() returns a long
        x1 = random(0,2000) - 1000;
        x1 *= 0.001;
        x2 = random(0,2000) - 1000;
        x2 *= 0.001;
        w = (x1 * x1) + (x2 * x2);

    } while( w >= 1.0 );

    w = sqrt( (-2.0 * log( w ) )/w );
    y = x1 * w;

    return mean + y * sd;

}

// Buzzer when finish each movement
void play_tone()
{
    //Play tone
    analogWrite(BUZZER_PIN, 127);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
}

void print(String str, float data)
{
    Serial.print(str);
    Serial.println(data);
}

byte BoyerMooreMajority(byte A[], byte n)
{
    //int confidence = 0; // РєРѕР»РёС‡РµСЃС‚РІРѕ Р»СЋРґРµР№, РЅРµ РЅР°С€РµРґС€РёС… РїР°СЂС‹ Рё РѕСЃС‚Р°РІС€РёС…СЃСЏ СЃС‚РѕСЏС‚СЊ
    // m stores majority element (if present)
    int m = -1;

    // initalize counter i with 0
    int i = 0;

    // do for each element A[j] of the array
    for (int j = 0; j < n; j++)
    {
        // If the counter i becomes 0, we set the current candidate
        // to A[j] and reset the counter to 1
        if (i == 0)
            m = A[j], i = 1;

            // If the counter is not 0, we increment or decrement the counter
            // according to whether A[j] is the current candidate
        else (m == A[j]) ? i++ : i--;
    }

    return m;
}



