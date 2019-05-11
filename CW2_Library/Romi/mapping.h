#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>
#include <Arduino.h>

const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = 0;//'#';
const int MAP_X=1800;
const int MAP_Y=1800;
const int threshold = 15;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        //void updateMapFeature(byte feature, int y, int x, int curr_y = 0, int curr_x = 0 );
        void updateMapFeature(byte feature, float y, float x, float curr_y = 0, float curr_x = 0, int change = 0 );
        //void  findUncoveredCenter();
        int  getUncoverCentre(int &x, int &y);
        float getAngleTO(int x_targ, int y_targ, int x_curr, int y_curr );

        int  indexToPose(int i, int map_size, int resolution);
        //int  poseToIndex(int x, int map_size, int resolution);
        int  poseToIndex(float x, int map_size, int resolution);

        char readMap(float y, float x);
        int x_uncovCenter;
        int y_uncovCenter;

    private:
        int X_size;
        int Y_size;
};

void Mapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
                
            }
        }
    }

}

void Mapper::printMap()
{
    int numVisitedGrids=0;
    Serial.println("Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value = EEPROM.read(eeprom_address);

            /*==========================================*/
            char value_char;
            if (value >= threshold-5) {value_char='O';}
            else if (value == 0) {value_char='#';}
            else if (value == 1) {value_char='L';}
            else if (value == 2) {value_char='R';}
            else if (value >= 3 && value < 10) {value_char=' ';}
            Serial.print(value_char);
            /*==========================================*/

//            Serial.print( (char)value );
//            Serial.print( value );
            Serial.print(" ");
            //====================================================
            if (value!=0)
              numVisitedGrids++;
            //====================================================
        }
        Serial.println("");
    }

    //====================================================
    Serial.println("======================");
    Serial.print("COVERAGE RATE = ");
    Serial.print((float)numVisitedGrids*100/(MAP_RESOLUTION*MAP_RESOLUTION));
    Serial.println("%");
    Serial.println("======================");
    //====================================================
  
}

float Mapper::getAngleTO(int x_targ, int y_targ, int x_curr, int y_curr )
{
    float angle = atan2(y_targ-y_curr,x_targ-x_curr)*180/PI;
//    if(angle < 180){
//      angle = -angle;
//    }
    return angle;
}

int Mapper::getUncoverCentre(int &x, int &y)
{
    int uncover_num = 0;
    float average_x = 0.0f;
    float average_y = 0.0f;
    for(int i = 0; i < MAP_RESOLUTION; i++){
        for(int j = 0; j < MAP_RESOLUTION; j++){
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value = EEPROM.read(eeprom_address);
            if(value == 0){
                uncover_num++;
                average_x += j;
                average_y += i;
            }
        }
    }
    x = (int)(average_x/uncover_num+0.5);
    y = (int)(average_y/uncover_num+0.5);
    if(x < 0 || x >= MAP_RESOLUTION || y < 0 || y >= MAP_RESOLUTION){
        return -1;  //error
    }
    else{
        return 1;   //success
    }
}

char Mapper::readMap(float y, float x)
{
    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);
    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;
    return EEPROM.read(eeprom_address);
}


/*
int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    return x / (map_size / resolution);
}
*/

int Mapper::poseToIndex(float x, int map_size, int resolution)
{
  return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}

/*
void Mapper::updateMapFeature(byte feature, float y, float x, float curr_y = 0, float curr_x = 0)
{
    updateMapFeature( feature, (int)y, (int)x, (int)curr_y, (int)curr_x );
}
*/

//void Mapper::updateMapFeature(byte feature, int y, int x, int curr_y = 0, int curr_x = 0)
void Mapper::updateMapFeature(byte feature, float y, float x, float curr_y = 0, float curr_x = 0, int change = 0)
{
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
      Serial.println(F("Error:Invalid co-ordinate"));
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);

    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  

    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {
//        EEPROM.update(eeprom_address, feature);
        if ((char)feature == 'O') {
          int new_value = EEPROM.read(eeprom_address) + change;
          new_value = constrain (new_value, 3, threshold);
          EEPROM.update(eeprom_address, new_value);
        } else {
          EEPROM.update(eeprom_address, change);
        }
    }
}

#endif
