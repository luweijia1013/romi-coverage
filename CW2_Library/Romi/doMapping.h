//
// Created by bulat on 23.04.19.
//

#include "irproximity.h"

#ifndef CW2_DOMAPPING_H
#define CW2_DOMAPPING_H

#endif //CW2_DOMAPPING_H

extern Kinematics Pose;
extern SharpIR centreDistanceSensor;
extern SharpIR DistanceSensor;
extern SharpIR DistanceSensorRight;
extern Mapper Map;
static unsigned long timeStart = millis();

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* This function groups up our sensor checks, and then
        * encodes into the map.  To get you started, we are
        * simply placing a character into the map.  However,
* you might want to look using a bitwise scheme to
        * encode more information.  Take a look at mapping.h
* for more information on how we are reading and
        * writing to eeprom memory.
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void doMapping() {

    float currentX = Pose.getX();
    float currentY = Pose.getY();
    float currentTheta = Pose.getThetaRadians();


    // Read the IR Sensor and determine distance in
    // mm.  Make sure you calibrate your own code!
    // We threshold a reading between 40mm and 12mm.
    // The rationale being:
    // We can't trust very close readings or very far.
    // ...but feel free to investigate this.


    float leftDistance = DistanceSensor.getDistanceInMM();
    DistanceSensor.findChangeSpeed();

    if (leftDistance > 80 && leftDistance < 400 && DistanceSensor.changeSpeed < 10 )
    {
        // We know the romi has the sensor mounted
        // to the front of the robot.  Therefore, the
        // sensor faces along Pose.Theta.
        // We also add on the distance of the
        // sensor away from the centre of the robot.
        leftDistance += (CHASSIS_RADIUS);        //5mm means the thickness of IR sensor plastic

        // Using Histogram method, we update all grids between the Romi's center and an Obstacle
        float sample_step = 51;         // 51 means half of a grid diagonal
        for(float i = leftDistance; i >= 0; i-=sample_step){

           float projected_x = currentX + ( i * cos( currentTheta + ANGLE_BETWEEN_IR_SENSORS ) );
           float projected_y = currentY + ( i * sin( currentTheta + ANGLE_BETWEEN_IR_SENSORS ) );
           if(i==leftDistance){
             //we increment grid corresponding to an obstacle by (+3)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, 3 );
           }
           else{
             //we decrement all grids between Romi center and obstacle by (-1)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, -1 );
           }
        }
    }


    float centerDistance = centreDistanceSensor.getDistanceInMM();
    centreDistanceSensor.findChangeSpeed();

    if (centerDistance > 80 && centerDistance < 400 && centreDistanceSensor.changeSpeed < 20 )
    {
        // We know the romi has the sensor mounted
        // to the front of the robot.  Therefore, the
        // sensor faces along Pose.Theta.
        // We also add on the distance of the
        // sensor away from the centre of the robot.
        centerDistance += (CHASSIS_RADIUS);        //5mm means the thickness of IR sensor plastic

        // Using Histogram method, we update all grids between the Romi's center and an Obstacle
        float sample_step = 51;         // 51 means half of a grid diagonal
        for(float i = centerDistance; i >= 0; i-=sample_step){

           float projected_x = currentX + ( i * cos( currentTheta ) );
           float projected_y = currentY + ( i * sin( currentTheta ) );
           if(i==centerDistance){
             //we increment grid corresponding to an obstacle by (+3)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, 3 );
           }
           else{
             //we decrement all grids between Romi center and obstacle by (-1)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, -1 );
           }
        }


/*        // Here we calculate the actual position of the obstacle we have detected
        float projected_x = currentX + ( centerDistance * cos( Pose.getThetaRadians() ) );
        float projected_y = currentY + ( centerDistance * sin( Pose.getThetaRadians() ) );
        Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, 1 );
*/
    }


    float rightDistance = DistanceSensorRight.getDistanceInMM();
    DistanceSensorRight.findChangeSpeed();

    if (rightDistance > 80 && rightDistance < 400 && DistanceSensorRight.changeSpeed < 10 )
    {
        // We know the romi has the sensor mounted
        // to the front of the robot.  Therefore, the
        // sensor faces along Pose.Theta.
        // We also add on the distance of the
        // sensor away from the centre of the robot.
        rightDistance += (CHASSIS_RADIUS);        //5mm means the thickness of IR sensor plastic

        // Using Histogram method, we update all grids between the Romi's center and an Obstacle
        float sample_step = 51;         // 51 means half of a grid diagonal
        for(float i = rightDistance; i >= 0; i-=sample_step){

           float projected_x = currentX + ( i * cos( currentTheta - ANGLE_BETWEEN_IR_SENSORS ) );
           float projected_y = currentY + ( i * sin( currentTheta - ANGLE_BETWEEN_IR_SENSORS ) );
           if(i==rightDistance){
             //we increment grid corresponding to an obstacle by (+3)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, 3 );
           }
           else{
             //we decrement all grids between Romi center and obstacle by (-1)
             Map.updateMapFeature( (byte)'O', projected_y, projected_x, currentY, currentX, -1 );
           }
        }
    }
/*
if ( millis() - timeStart > 3000)
{
    timeStart = millis();
    print("distance: ", rightDistance);
    print("changeSpeed: ", DistanceSensorRight.changeSpeed );
    Map.printMap();
}
*/
    // Check RFID scanner.
    // Look inside RF_interface.h for more info.
    if (checkForRFID())
    {

        // Here we calculate the actual position of the obstacle we have detected
        float projected_x = currentX + ( RFID_SENSOR_SEPARATION * cos( currentTheta ) );
        float projected_y = currentY + ( RFID_SENSOR_SEPARATION * sin( currentTheta ) );
        // Add card to map encoding.
        Map.updateMapFeature( (byte)'R', projected_y, projected_x );

        // you can check the position reference and
        // bearing information of the RFID Card in
        // the following way:
        // serialToBearing( rfid.serNum[0] );
        // serialToXPos( rfid.serNum[0] );
        // serialToYPos( rfid.serNum[0] );
        //
        // Note, that, you will need to set the x,y
        // and bearing information in rfid.h for your
        // experiment setup.  For the experiment days,
        // we will tell you the serial number and x y
        // bearing information for the cards in use.

    }

/*        // Line detected
    else if( LineLeft.readRaw() > 580 or LineRight.readRaw() > 580 )
    {
        // Here we calculate the actual position of the obstacle we have detected
        float projected_x = currentX + ( LINE_SENSOR_SEPARATION * cos( currentTheta));
        float projected_y = currentY + ( LINE_SENSOR_SEPARATION * sin( currentTheta));
        Map.updateMapFeature( (byte)'L', projected_y, projected_x );
    }
*/
        // Recording as visited grid
    //else if( (byte)Map.readMap(currentY, currentX) == (byte)'#' || (byte)Map.readMap(currentY, currentX) == (byte)'O')
    else if( (byte)Map.readMap(currentY, currentX) != 3)
    {
        Map.updateMapFeature( (byte)' ', currentY, currentX, 0, 0, 3 );
    }


}
// end of mapping

