//
// Created by bulat on 23.04.19.
//

#include "irproximity.h"
#include "motions.h"

#ifndef CW2_OBJECTAVOID_H
#define CW2_OBJECTAVOID_H

#endif //CW2_OBJECTAVOID_H

#define IRMiddle A4
#define IRLeft A0
#define IRRight A3

extern Kinematics Pose;

//SharpIR centreDistanceSensor(IRMiddle, 84.081, -0.702, 6986.1, -0.721 ); //central Distance sensor
SharpIR centreDistanceSensor(IRMiddle, 87.553, -0.712, 6986.1, -0.721 ); //central Distance sensor   //CALIBRATED without lowpass
//SharpIR centreDistanceSensor(IRMiddle, 117.51, -0.802, 8514.4, -0.795); //central Distance sensor   // CALIBRATION WITHOUT LOWPASS
SharpIR DistanceSensor(IRLeft,86.492, -0.708, 6986.1, -0.721); //left Distance sensor
SharpIR DistanceSensorRight(IRRight, 94.576, -0.744, 8514.4, -0.795); //left Distance sensor

float Kp_dist = 0.02; //Proportional gain for position controller
float Kd_dist= 0.00; //Derivative gain for position controller
float Ki_dist= 0.0000; //Integral gain for position controller

PID keep_dist(Kp_dist, Kd_dist, Ki_dist);

unsigned long followStartTime;
double followStartX;
double followStartY;
byte l_power;
byte r_power;

void printIRSensors()
{
    Serial.print("Sensors: ");
    Serial.print(DistanceSensor.getDistanceInMM());
    Serial.print("\t\t");
    Serial.print(centreDistanceSensor.getDistanceInMM());   //read_calibrated()); //
    Serial.print("\t\t");
    Serial.println(DistanceSensorRight.getDistanceInMM());
}

void follow_obstacle()
{

    float dist = DistanceSensor.getDistance();
    float centre_dist = centreDistanceSensor.getDistance();

    if (centre_dist<200 or dist<150)
    {
        l_power = 0;
        r_power = 0;
        rotate (deg2rad(15), 1);
    }
    else
    {
        l_power = default_power;
        r_power = default_power;
    }


    float output = keep_dist.update(150,dist);

    //======================================================================
    // FOR SPEED CONTROLLER
    float l_power_demand = (l_power) + output;
    float r_power_demand = (r_power) - output;

    applyMotorInputs(l_power_demand,r_power_demand);
    delay(10);

    //======================================================================

    if (millis()-followStartTime > 20000 and Pose.getDistanceFrom(followStartX, followStartY)<100)
    {
        mode=0;
        play_tone();
        clear_var();
        use_speed_controller = true;
    }
}

