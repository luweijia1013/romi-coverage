#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>

const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const int MAP_X=1800;
const int MAP_Y=1800;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);
    
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

    Serial.println("Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)value );
            Serial.print(" ");
        }
        Serial.println("");
    }
  
}

char Mapper::readMapFeatureByIndex(int ind_y, int ind_x)
{
    if (ind_x > MAP_X || ind_x < 0 || ind_y > MAP_Y || ind_y < 0)
    {
      //Serial.println(F("Error:Invalid co-ordinate"));
      return 'e';
    }
    int eeprom_address = (ind_x * MAP_RESOLUTION) + ind_y;  
    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
        return 'e';
    }
    else
    {
        byte value = EEPROM.read(eeprom_address);
        return (char)value;
    }
    
}

void Mapper::checkMap()
{
    fillObstacle();
}

void Mapper::fillObstacle()
{
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            if(value == '#'){
                bool obstacle_inside[MAP_RESOLUTION][MAP_RESOLUTION] = {false};
                bool obstacle_visited[MAP_RESOLUTION][MAP_RESOLUTION] = {false};
                //std::vector<int> obstacle_inside;
                dfs(& obstacle_inside,& obstacle_visited, j, i);
                for(int m=0;m<MAP_RESOLUTION;,m++){
                    for(int n=0;n<MAP_RESOLUTION;n++){
                        if(obstacle_inside[n][m]){

                        }
                    }
                }
            }
        }
    }
}

int Mapper::dfs(bool (&obstacle_inside)[MAP_RESOLUTION][MAP_RESOLUTION],bool (&obstacle_visited)[MAP_RESOLUTION][MAP_RESOLUTION], int j, int i){
    if(obstacle_visited[j][i] || obstacle_inside[j][i]){
        return 1;
    }
    if(readMapFeatureByIndex(j,i) != '#' && readMapFeatureByIndex(j,i) != 'O'){
        return -1;
    }
    if(readMapFeatureByIndex(j,i) == '#'){
        obstacle_visited[j][i] = true;
        if(dfs(obstacle_inside,j-1,i) < 0){
            return -1;
        }
        if(dfs(obstacle_inside,j+1,i) < 0){
            return -1;
        }
        if(dfs(obstacle_inside,j,i+1) < 0){
            return -1;
        }
        if(dfs(obstacle_inside,j,i-1) < 0){
            return -1;
        }
        obstacle_inside[j][i] = true;
        return 1;
    }
    if(readMapFeatureByIndex(j,i) == 'O'){
        return 1;
    }
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );  
}

void Mapper::updateMapFeature(byte feature, int y, int x)
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
        EEPROM.update(eeprom_address, feature);
    }
        

}

void Mapper::updateMapFeature(byte feature, int address)
{
    if (adress < 0 || address > 1023)
    {
      Serial.println(F("Error:Invalid co-ordinate"));
      return;
    }
    EEPROM.update(address, feature);

}


#endif
