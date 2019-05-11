//
// Created by bulat on 23.04.19.
//

#ifndef CW2_MOTIONS_H
#define CW2_MOTIONS_H

#endif //CW2_MOTIONS_H

#include "doMapping.h"


extern byte l_power;
extern byte r_power;
extern int default_power;

int default_power = 25;
bool l_dir = false;
bool r_dir = false;

// clearing global variables between movements
void clear_var()
{
    // Send speeds to pins, to motor drivers.
    analogWrite( MOTOR_PWM_L, 0 );
    analogWrite( MOTOR_PWM_R, 0 );
    // encoder's variables
    left_encoder_count = 0;
    right_encoder_count = 0;
    last_count_left = 0;
    last_count_right = 0;
    // timer3 variables for speed calculation
    Pose.last_left_encoder_count=0;
    Pose.last_right_encoder_count=0;
    // Set initial l_power and r_power values.
    l_power = default_power;
    r_power = default_power;
    //==================================
    LeftSpeedControl.reset();
    RightSpeedControl.reset();
    left_speed_demand = 5;
    right_speed_demand = 5;
}

// this function allows motors to catch up each other when lagging/leading
// anti systematic error
void calibrate(int l_counts, int r_counts)
{
    int wheels_difference = abs(l_counts) - abs(r_counts); // if left is front of right tresh

    int buff = round( ((float) r_power - wheels_difference*0.2));
    if (buff > 0 and buff < 255)
    {
        l_power = buff;
    }
}

// just run motors on specified power and direction
void applyMotorInputs(float out_L, float out_R)
{
    digitalWrite(MOTOR_DIR_L, out_L>=0 ? LOW: HIGH);
    digitalWrite(MOTOR_DIR_R, out_R>=0 ? LOW: HIGH);

    out_L=constrain(abs(out_L),0,100);
    out_R=constrain(abs(out_R),0,100);
    analogWrite(MOTOR_PWM_L, out_L);
    analogWrite(MOTOR_PWM_R, out_R);
}

void rotate(float rotation_ang, bool direct)
{
    clear_var();

    // direction of motors when rotating
    if (direct == 1)   // right
    {digitalWrite(MOTOR_DIR_L, l_dir);
        digitalWrite(MOTOR_DIR_R, !r_dir);}
    else              // left
    {digitalWrite(MOTOR_DIR_L, !l_dir);
        digitalWrite(MOTOR_DIR_R, r_dir);}

    int rotat_pulse_counts = abs( ((WHEEL_DISTANCE/2)*rotation_ang)/MM_PER_COUNT );   // (pi/2)*wheel_radius

    // Straigth line motion within a certain numbers of pulses that gives 10cm
    while ( (abs(left_encoder_count)+abs(right_encoder_count))/2 < rotat_pulse_counts )
    {
        Pose.update();
        doMapping();
        // Send speeds to pins, to motor drivers.
        analogWrite(MOTOR_PWM_L, l_power);
        analogWrite(MOTOR_PWM_R, r_power);
        calibrate(left_encoder_count, right_encoder_count);
    }

    clear_var();
}

// moving along straigth line
void move_straigth(
                  int distance,
                  bool direct
                  )
{
  clear_var();  

  if (direct == 1)   // forward
    {digitalWrite(MOTOR_DIR_L, l_dir);
     digitalWrite(MOTOR_DIR_R, r_dir);}
  else              // backward
    {digitalWrite(MOTOR_DIR_L, !l_dir);
     digitalWrite(MOTOR_DIR_R, !r_dir);}

  int dist_pulse_counts = round(distance/0.15271527);

  // Straigth line motion within a certain numbers of pulses that gives 10cm
  while (    (abs(left_encoder_count)+abs(right_encoder_count))/2 < dist_pulse_counts )
    {
      Pose.update();
      doMapping();
      // Send speeds to pins, to motor drivers.
      analogWrite( MOTOR_PWM_L, l_power );
      analogWrite( MOTOR_PWM_R, r_power );
      calibrate(left_encoder_count, right_encoder_count);
//      delay(50);
    }

  clear_var();
}

// Bulat written
float simplifyAngle(float angle)
{
    while (angle > 180)
    {
        angle -= 360;
    }
    while (angle < -180)
    {
        angle += 360;
    }

    return angle;
}

void directAnyAngle(float desiredAngle)
{

    float turningAngle = simplifyAngle(desiredAngle - Pose.getThetaDegrees());
    turningAngle = deg2rad(turningAngle);
    Serial.print("direct!!!!");
    Serial.println(rad2deg(turningAngle));
    //turnAnyAngle(rad2deg(turningAngle));
    use_speed_controller=false;

    if (turningAngle > 0)
    {
        rotate(abs(turningAngle), 0);
    }
    else
    {
        rotate(abs(turningAngle), 1);
    }
    use_speed_controller = true;
}


