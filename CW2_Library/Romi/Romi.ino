
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Library Includes.                                                             *
 * Be sure to check each of these to see what variables/functions are made        *
 * global and accessible.                                                        *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "motor.h"
#include "pid.h"
#include "interrupts.h"

#include "line_sensors.h"
//#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"
#include "objectAvoid.h"
//#include "pins.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600
#define LOOP_DELAY 10


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
//LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 1.5, 0, 0.001 );

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/*================================================================================================*/
float Kp_sp = 3.5; //Proportional gain for position controller
float Kd_sp= 20.9; //Derivative gain for position controller
float Ki_sp= 0.04; //Integral gain for position controller

PID   l_keep_sp(Kp_sp, Kd_sp, Ki_sp);
PID   r_keep_sp(Kp_sp, Kd_sp, Ki_sp);

float Kp_line = 0.1; //Proportional gain for position controller
float Kd_line = 0; //Derivative gain for position controller
float Ki_line = 0; //Integral gain for position controller

PID keep_line(Kp_line, Kd_line, Ki_line);

// modes: // 0-calibrate // 1-find line // 2-follow the line // 3-go home
int mode;
static unsigned long turnToUncover;
static unsigned long InUncoverAlgo = -1;
static unsigned long awayFromBorder;

/*================================================================================================*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
float left_speed_demand = 0;
float right_speed_demand = 0;


//variables for borders detecion
boolean upperBorder = false;
boolean bottomBorder = false;
boolean leftBorder = false;
boolean rightBorder = false;
int borderThickness = 70;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This setup() routine initialises all class instances above and peripherals.   *
 * It is recommended:                                                            *
 * - You keep this sequence of setup calls if you are to use all the devices.    *
 * - Comment out those you will not use.                                         *
 * - Insert new setup code after the below sequence.                             *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
void setup() {

    // These two function set up the pin
    // change interrupts for the encoders.
    setupLeftEncoder();
    setupRightEncoder();
    startTimer();
    //unsigned long timeStart = 0;

            //Set speed control maximum outputs to match motor
    LeftSpeedControl.setMax(100);
    RightSpeedControl.setMax(100);

    // For this example, we'll calibrate only the
    // centre sensor.  You may wish to use more.
//  LineCentre.calibrate();
    LineLeft.calibrate();
    LineRight.calibrate();

    //Setup RFID card
    setupRFID();

    // These functions calibrate the IMU and Magnetometer
    // The magnetometer calibration routine require you to move
    // your robot around  in space.
    // The IMU calibration requires the Romi does not move.
    // See related lab sheets for more information.
    /*
    Wire.begin();
    Mag.init();
    Mag.calibrate();
    Imu.init();
    Imu.calibrate();
    */

    // Set the random seed for the random number generator
    // from A0, which should itself be quite random.
    randomSeed(analogRead(A0));


    // Initialise Serial communication
    Serial.begin( BAUD_RATE );
    delay(1000);
    Serial.println("Board Reset");

    // Romi will wait for you to press a button and then print
    // the current map.
    //
    // !!! A second button press will erase the map !!!
    ButtonB.waitForButton();

    Map.printMap();

    // Watch for second button press, then begin autonomous mode.
    ButtonB.waitForButton();

    Serial.println("Map Erased - Mapping Started");
    Map.resetMap();
    // Map.initialTestMap();

    // Your extra setup code is best placed here:
    // ...
    // ...
    // but not after the following:

    l_power = default_power;
    r_power = default_power;
    mode = 0;
    //Pose.setDebug(1);
    turnToUncover = millis();

    // Because code flow has been blocked, we need to reset the
    // last_time variable of the PIDs, otherwise we update the
    // PID with a large time elapsed since the class was
    // initialised, which will cause a big intergral term.
    // If you don't do this, you'll see the Romi accelerate away
    // very fast!
    LeftSpeedControl.reset();
    RightSpeedControl.reset();
    left_speed_demand = 5;
    right_speed_demand = 5;

}

void doMovement();
void checkForBorder();
void moveAwayFromBorder();
int sign=1;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This loop() demonstrates all devices being used in a basic sequence.
 * The Romi should:
 * - move forwards with random turns
 * - log lines, RFID and obstacles to the map.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {

    // Remember to always update kinematics!!
    Pose.update();
    // and do Mapping
    doMapping();
//    printIRSensors();

/*  sign = !sign;
  rotate( deg2rad(60), sign);
  delay(100);
  move_straigth( 100, sign);
*/


//    print("Mode: ", mode);
    switch (mode)
    {

        case 0:
            doMovement();
            break;

        /*case 1:
            // Finding the line
            follow_obstacle();
            break;

        case 2:
            follow_line();
            break;*/

        case 3:
            moveAwayFromBorder();
            break;

        default:
            // statements
            break;
    }


    delay(10);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * We have implemented a random walk behaviour for you
 * with a *very* basic obstacle avoidance behaviour.
 * It is enough to get the Romi to drive around.  We
 * expect that in your first week, should should get a
 * better obstacle avoidance behaviour implemented for
 * your Experiment Day 1 baseline test.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void doMovement() {

    // Static means this variable will keep
    // its value on each call from loop()
    static unsigned long walk_update = millis();

    // used to control the forward and turn
    // speeds of the robot.
    float forward_bias;
    float turn_bias;

    checkForBorder();
    // if (millis()-turnToUncover>10000)
    // {
    //     turnToUncover = millis();
    //     play_tone();
    //     // turnToUncoveredArea();
    //     // Serial.print(Pose.getX());
    //     // Serial.print("  ");
    //     // Serial.println(Pose.getY()); 
    //     directAnyAngle(180);
    //     Serial.println("biiiiiiiiiii!!!!!!!!"); 
    //     // Serial.print(Pose.getX());
    //     // Serial.print("  ");
    //     // Serial.println(Pose.getY()); 
    //     return;
    // }

    // Check if we are about to collide.  If so,
    // zero forward speed
    //print("Center distance: ", centreDistanceSensor.getDistance());

    if(centreDistanceSensor.getDistanceInMM() <= 200 || DistanceSensorRight.getDistanceInMM()<= 200 || DistanceSensor.getDistanceInMM() <= 200) {
        forward_bias = 0;
        turn_bias = randGaussian(0, 5);; //2.5;
    }
    /*else if (LineCentre.readRaw()>500)
    {
    }
    else if (LineLeft.readRaw()>500 or LineRight.readRaw()>500)
    {
        forward_bias = 0;
        turn_bias = 0;
        mode=2;
        use_speed_controller = false;

    }*/
    else
    {
        forward_bias = 5;
        /*** uncover_gaussian_algorithm ***/
        float mean_gaussian = 0;
        float stand_devi = 3.5;
        if(millis() - turnToUncover > 6000){
            if(InUncoverAlgo < 0){
                InUncoverAlgo = millis();
            }
            else if(millis() - InUncoverAlgo < 15000){
                int x_ind_target;
                int y_ind_target;
                if(Map.getUncoverCentre(x_ind_target, y_ind_target) > 0){
                    int x_ind_curr = Map.poseToIndex(Pose.getX(), MAP_X, MAP_RESOLUTION);
                    int y_ind_curr = Map.poseToIndex(Pose.getY(), MAP_Y, MAP_RESOLUTION);
                    float angle = Map.getAngleTO (x_ind_target, y_ind_target, x_ind_curr, y_ind_curr);
                    float turningAngle = simplifyAngle(angle - Pose.getThetaDegrees());
                    mean_gaussian = -deg2rad(turningAngle);
                    stand_devi = 3.5;
                    Map.printMap();
                    Serial.print(x_ind_target);
                    Serial.println(y_ind_target);
                } 
                else{
                    Serial.println('GET UNCOVER CENTRE ERROR');
                }
            }
            else if(millis() - InUncoverAlgo > 30000){
                InUncoverAlgo = millis();
            }
            else{
                mean_gaussian = 0;
                stand_devi = 3.5;
                Serial.println("***");
            }
            
        }
        turn_bias = randGaussian(mean_gaussian, stand_devi);
        Serial.print(Pose.getX());
        Serial.print("  ");
        Serial.println(Pose.getY());        
    }


    // Periodically set a random turn.
    // Here, gaussian means we most often drive
    // forwards, and occasionally make a big turn.
    if( millis() - walk_update > 500 ) {
        walk_update = millis();

        // randGaussian(mean, sd).  utils.h
        //turn_bias = randGaussian(0, 3.5);

        // Setting a speed demand with these variables
        // is automatically captured by a speed PID
        // controller in timer3 ISR. Check interrupts.h
        // for more information.
        left_speed_demand = forward_bias + turn_bias;
        right_speed_demand = forward_bias - turn_bias;
    }

    

}

void turnToUncoveredArea()
{
  int x_ind_target;
  int y_ind_target;
  int result = Map.getUncoverCentre(x_ind_target, y_ind_target);
  int x_ind_curr = Map.poseToIndex(Pose.getX(), MAP_X, MAP_RESOLUTION);
  int y_ind_curr = Map.poseToIndex(Pose.getY(), MAP_Y, MAP_RESOLUTION);
  Serial.println("before turn:");
  Serial.println(x_ind_curr);
  Serial.println(y_ind_curr);
  Serial.println(x_ind_target);
  Serial.println(y_ind_target);
  float angle = Map.getAngleTO (x_ind_target, y_ind_target, x_ind_curr, y_ind_curr);
  Serial.println("before direct:");
  Serial.println(angle);
  directAnyAngle(angle);
}

void checkForBorder()
{
    if(millis() - awayFromBorder < 5000){
        //avoid multiple action continuesly for border
        return;
    }
    // checking if We are close to border
    // Additionally changing mode to 2!!!.
    float currentX = Pose.getX();
    float currentY = Pose.getY();

    if (currentX < borderThickness)
    {
//        play_tone();
        bottomBorder=true;
        mode = 3;
    }
    if (currentX > 1800 - borderThickness)
    {
//        play_tone();
        upperBorder = true;
        mode = 3;
    }
    if (currentY < borderThickness)
    {
//        play_tone();
        leftBorder = true;
        mode = 3;
    }
    if (currentY > 1800 - borderThickness)
    {
//        play_tone();
        rightBorder = true;
        mode = 3;
    }
    if(mode == 3){
        awayFromBorder = millis();
        moveAwayFromBorder();
    }

}

void moveAwayFromBorder()
{
    //play_tone();
    if (upperBorder && leftBorder)
    {
        directAnyAngle(135);
    }
    else if (upperBorder && rightBorder)
    {
        directAnyAngle(-135);
    }
    else if (bottomBorder && leftBorder)
    {
        directAnyAngle(45);
    }
    else if (bottomBorder && rightBorder)
    {
        directAnyAngle(-45);
    }
    else if (bottomBorder)
    {
        directAnyAngle(0);
    }
    else if (upperBorder)
    {
        directAnyAngle(180);
    }
    else if (leftBorder)
    {
        directAnyAngle(90);
    }
    else if (rightBorder)
    {
        directAnyAngle(-90);
    }

    //runFW(500);
    upperBorder = false;
    bottomBorder = false;
    rightBorder = false;
    leftBorder = false;
    mode = 0;
}



//Sensor fusion for Drive controller
double Location()
{
    // calibrated sensor readings
    double left_sensor = LineLeft.readRaw();
//  double centre_sensor = LineCentre.readRaw();
    double rigth_sensor = LineRight.readRaw();
//  double total = left_sensor + centre_sensor +rigth_sensor;
    double total = left_sensor +rigth_sensor;

    // calculating probabilities
    double P_left = left_sensor/total;
//  double P_centre = centre_sensor/total;
    double P_rigth = rigth_sensor/total;

    // Sensor fusion
//  double Location = (P_left*1000)+(P_centre*2000)+(P_rigth*3000);
    double Location = (P_left*1000)+(P_rigth*3000);

    return (Location-2000);
}

// average value of all line following sensors
float read_all_sensors()
{
//  return (LineLeft.readRaw()+LineCentre.readRaw()+LineRight.readRaw())/3;
    return (LineLeft.readRaw()+LineRight.readRaw())/2;
}

void follow_line()
{
    // power controller
    float changeInTurn = keep_line.update((float)0, (float) Location());

    float default_sp_L = (default_power) - changeInTurn;
    float default_sp_R = (default_power) + changeInTurn;

    applyMotorInputs(default_sp_L, default_sp_R);
    delay(LOOP_DELAY);

    // Final stop as line finishes
    if ( read_all_sensors()<330)
    {
        clear_var();
        play_tone();
        mode=0;
        use_speed_controller = true;
    }

}
