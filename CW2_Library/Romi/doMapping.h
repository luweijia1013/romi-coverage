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

    // Read the IR Sensor and determine distance in
  // mm.  Make sure you calibrate your own code!
  // We threshold a reading between 40mm and 12mm.
  // The rationale being:
  // We can't trust very close readings or very far.
  // ...but feel free to investigate this.
  float distance = DistanceSensor.getDistanceInMM();
  if( distance < 40 && distance > 12 ) {
    //Frank: should the distance above be more accurate?

    // We know the romi has the sensor mounted
    // to the front of the robot.  Therefore, the
    // sensor faces along Pose.Theta.
    // We also add on the distance of the 
    // sensor away from the centre of the robot.
    distance += 80;


    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( distance * cos( Pose.getThetaRadians() ) );
    float projected_y = Pose.getY() + ( distance * sin( Pose.getThetaRadians() ) );
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );
    
    
  } 

  // Check RFID scanner.
  // Look inside RF_interface.h for more info.
  if( checkForRFID() ) {

    // Add card to map encoding.  
    Map.updateMapFeature( (byte)'R', Pose.getY(), Pose.getX() );

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

  // Basic uncalibrated check for a line.
  // Students can do better than this after CW1 ;)
  // else if( LineCentre.readRaw() > 580 ) {
  //     Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
  // } 
  
  else{
    Map.updateMapFeature((byte)' ', Pose.getY(), Pose.getX());
  }


}
// end of mapping

