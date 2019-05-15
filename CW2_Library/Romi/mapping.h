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
        void initialTestMap();
        void printMap();
        char readMapFeatureByIndex(int ind_x, int ind_y);
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        void updateMapFeature(byte feature, int address);
        void checkMap();
        int  getUncoverCentre(int &x, int &y);
        void fillObstacle();
        
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);
        int  dfs(bool **obstacle_inside,bool (&obstacle_outside)[MAP_RESOLUTION][MAP_RESOLUTION], int j, int i);
        float getAngleTO(int x_targ, int y_targ, int x_curr, int y_curr );

    
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

void Mapper::initialTestMap()
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
                if(eeprom_address == 26 || eeprom_address == 27 || eeprom_address == 28 || 
                eeprom_address == 50 || eeprom_address == 53 || eeprom_address == 75 ||
                eeprom_address == 76 || eeprom_address == 78 || eeprom_address == 102 || eeprom_address == 103){
                    EEPROM.update(eeprom_address, 'O' );
                }
                else{
                    EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
                }
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
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)value );
            Serial.print(" ");
            if(value != '#'){
                numVisitedGrids++;
            }
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

char Mapper::readMapFeatureByIndex(int ind_x, int ind_y)
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

int Mapper::getUncoverCentre(int &x, int &y)
{
    int uncover_num = 0;
    float average_x = 0.0f;
    float average_y = 0.0f;
    for(int i = 0; i < MAP_RESOLUTION; i++){
        for(int j = 0; j < MAP_RESOLUTION; j++){
            if(readMapFeatureByIndex(i,j) == '#'){
                uncover_num++;
                average_x += i;
                average_y += j;
            }
        }
    }
    x = (int)(average_x/uncover_num+0.5);
    y = (int)(average_y/uncover_num+0.5);
    if(x < 0 || x >= MAP_RESOLUTION || y < 0 || y >= MAP_RESOLUTION){
        return -1;
        //error
    }
    else{
        return 1;
        //success
    }
}

void Mapper::fillObstacle()
{
    bool obstacle_outside[MAP_RESOLUTION][MAP_RESOLUTION];
    for(int i = 0;i<MAP_RESOLUTION;i++){
      for(int j=0;j<MAP_RESOLUTION;j++){
        obstacle_outside[i][j] = false;
      }
    }
    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
//           Serial.print(i);
//                     Serial.print(" ");
//                     Serial.print(j);
//                     Serial.print(" ");
//                     Serial.print(obstacle_outside[i][j]);
//                     Serial.println("&&");
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            if(value == '#' && !obstacle_outside[i][j]){
                bool **obstacle_inside = new bool*[MAP_RESOLUTION];
                for(int p=0;p<MAP_RESOLUTION;p++){
                    obstacle_inside[p] = new bool[MAP_RESOLUTION];
                    for(int q=0;q<MAP_RESOLUTION;q++){
                        obstacle_inside[p][q] = false;
                    }
                }
                // bool obstacle_inside[MAP_RESOLUTION][MAP_RESOLUTION] = {false};
                //std::vector<int> obstacle_inside;
                if(dfs(obstacle_inside, obstacle_outside, i, j) >= 0){
                    // Serial.print(i);
                    // Serial.print(" ");
                    // Serial.print(j);
                    // Serial.println("&&");
                    for(int m=0;m<MAP_RESOLUTION;m++){
                        for(int n=0;n<MAP_RESOLUTION;n++){
                            if(obstacle_inside[m][n]){
                                updateMapFeature('O',(m*MAP_RESOLUTION)+n);
                            }
                        }
                    }
                }
                for(int p=0;p<MAP_RESOLUTION;p++){
                  for(int q=0;q<MAP_RESOLUTION;q++){
                     obstacle_inside[p][q] = false;
                  }
                    delete[] obstacle_inside[p];
                }
                delete[] obstacle_inside;
            }
        }
    }
}

int Mapper::dfs(bool **obstacle_inside,bool (&obstacle_outside)[MAP_RESOLUTION][MAP_RESOLUTION], int i, int j){
//     Serial.print(i);
//     Serial.print(" ");
//     Serial.print(j);
//     Serial.println("!!");
    if(obstacle_inside[i][j]){
        return 1;
    }
    if(obstacle_outside[i][j]){
        return -1;
    }
    if(readMapFeatureByIndex(i,j) != '#' && readMapFeatureByIndex(i,j) != 'O'){
        return -1;
    }
    if(readMapFeatureByIndex(i,j) == '#'){
        obstacle_inside[i][j] = true;
        if(dfs(obstacle_inside, obstacle_outside,i+1,j) < 0){
            obstacle_outside[i][j] = true;
            return -1;
        }
        if(dfs(obstacle_inside, obstacle_outside,i-1,j) < 0){
            obstacle_outside[i][j] = true;
            return -1;
        }
        if(dfs(obstacle_inside, obstacle_outside,i,j+1) < 0){
            obstacle_outside[i][j] = true;
            return -1;
        }
        if(dfs(obstacle_inside, obstacle_outside,i,j-1) < 0){
            obstacle_outside[i][j] = true;
            return -1;
        }
        return 1;
    }
    if(readMapFeatureByIndex(i,j) == 'O'){
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
    if (address < 0 || address > 1023)
    {
      Serial.println(F("Error:Invalid co-ordinate"));
      return;
    }
    EEPROM.update(address, feature);

}

float Mapper::getAngleTO(int x_targ, int y_targ, int x_curr, int y_curr )
{
    float angle = atan2(y_targ-y_curr,x_targ-x_curr)*180/PI;
//    if(angle < 180){
//      angle = -angle;
//    }
    return angle;
}


#endif