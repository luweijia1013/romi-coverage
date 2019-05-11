//
// Created by bulat on 01.04.19.
//

#ifndef CW2_MOTOR_H
#define CW2_MOTOR_H

#endif //CW2_MOTOR_H

#define L_SPD 10
#define L_DIR 16
#define R_SPD 9
#define R_DIR 15

byte l_spd;
byte r_spd;

extern volatile long left_encoder_count;
extern volatile long right_encoder_count;

void setupMotors()
{
    // put your setup code here, to run once:
    pinMode(L_SPD, OUTPUT);
    pinMode(R_SPD, OUTPUT);

    pinMode(L_DIR, OUTPUT);
    pinMode(R_DIR, OUTPUT);

    l_spd = 0;
    r_spd = 0;
}

//change direction of motion
// to go forward
void setupMotorsForward()
{
    left_encoder_count=0;
    right_encoder_count=0;
    digitalWrite(L_DIR, LOW);
    digitalWrite(R_DIR, LOW);
}

//change direction of motion
// to go backward
void setupMotorsBackward()
{
    left_encoder_count=0;
    right_encoder_count=0;
    digitalWrite(L_DIR, HIGH);
    digitalWrite(R_DIR, HIGH);
}

int mmToEncoder(int mm)
{
    return round((float) mm/0.1572);
}

// this function adjust left and
// right motor speed to drive straight
// forward
void forward(int speed)
{
    int tresh = (left_encoder_count - right_encoder_count)*0.6;
    analogWrite(R_SPD, speed-tresh);
    analogWrite(L_SPD, speed);

}

// this function adjust left and
// right motor speed to drive straight
// backward
void backward(int speed)
{
    int tresh = (left_encoder_count - right_encoder_count)*0.6;
    analogWrite(R_SPD, speed+tresh);
    analogWrite(L_SPD, speed);

}

void turn(int speed)
{
    int tresh = (abs(left_encoder_count) - abs(right_encoder_count))*0.6;
    analogWrite(R_SPD, speed-tresh);
    analogWrite(L_SPD, speed);
}

void accelerate()
{
    setupMotorsForward();
    int avarage = (right_encoder_count+left_encoder_count)/2;
    //acceleration
    while (avarage<100)
    {
        int speed = avarage*0.5 + 10;
        forward(speed);
        delay(10);
        avarage = (right_encoder_count+left_encoder_count)/2;
    }
}

void deccelerate()
{
    //deseleration
    int avarage = (right_encoder_count+left_encoder_count)/2;
    int averageFinal = avarage + 100;
    while (avarage<averageFinal)
    {
        int speed = (averageFinal - avarage)*0.5;
        forward(speed);
        delay(5);
        avarage = (right_encoder_count+left_encoder_count)/2;

        if (avarage > averageFinal-2)
        {
            break;
        }
    }
}

void stop()
{
    analogWrite(R_SPD, 0);
    analogWrite(L_SPD, 0);
}
// Functions just for use of single motor
void runLeftMotor(int value)
{
    if(value>0)
    {
        digitalWrite(L_DIR, LOW);
        analogWrite(L_SPD, value);
    } else{
        digitalWrite(L_DIR, HIGH);
        analogWrite(L_SPD, abs(value));
    }
}

void runRightMotor(int value)
{
    if(value>0)
    {
        digitalWrite(R_DIR, LOW);
        analogWrite(R_SPD, value);
    } else{
        digitalWrite(R_DIR, HIGH);
        analogWrite(R_SPD, abs(value));
    }
}

// Functions for motion forward and
// and backward for desired distance.
void runFW(int distance)
{
    setupMotorsForward();
    int avarage = (right_encoder_count+left_encoder_count)/2;
    //acceleration
    while (avarage<100)
    {
        int speed = avarage*0.5 + 10;
        forward(speed);
        delay(10);
        avarage = (right_encoder_count+left_encoder_count)/2;
    }

    //motion
    int dist_temp = distance - 100;
    while (avarage<dist_temp)
    {
        forward(50);
        delay(50);
        avarage = (right_encoder_count+left_encoder_count)/2;
    }

    //deseleration
    while (avarage<distance)
    {
        int speed = (distance - avarage)*0.5;
        forward(speed);
        delay(5);
        avarage = (right_encoder_count+left_encoder_count)/2;

        if (avarage > distance-2)
        {
            break;
        }
    }

    //stop
    analogWrite(R_SPD, 0);
    analogWrite(L_SPD, 0);

}

void runBW(int distance)
{
    setupMotorsBackward();
    int avarage = abs((right_encoder_count+left_encoder_count)/2);
    //acceleration
    while (avarage<100)
    {
        int speed = avarage*0.5 + 10;
        backward(speed);
        delay(10);
        avarage = abs((right_encoder_count+left_encoder_count)/2);
    }

    //motion
    int dist_temp = distance - 100;
    while (avarage<dist_temp)
    {
        backward(50);
        delay(50);
        avarage = abs((right_encoder_count+left_encoder_count)/2);
    }

    //deseleration
    while (avarage<distance)
    {

        int speed = (distance - avarage)*0.5;
        backward(speed);
        delay(5);
        avarage = abs((right_encoder_count+left_encoder_count)/2);

        if (avarage > distance-2)
        {
            break;
        }
    }

    //stop
    analogWrite(R_SPD, 0);
    analogWrite(L_SPD, 0);
}


//Functions for turning robot
// around one still weal
void turnLeft(int degree)
{
    int distance = round(((float) degree*2*PI*145)/(0.1566*360));
    setupMotorsForward();
    //acceleration
    while (left_encoder_count<100)
    {
        int speed = left_encoder_count*0.5 + 10;
        runRightMotor(speed);
        delay(10);
    }

    //motion
    int dist_temp = distance - 100;
    while (left_encoder_count<dist_temp)
    {
        runRightMotor(50);
        delay(50);
    }

    //deseleration
    while (left_encoder_count<distance)
    {
        int speed = (distance - left_encoder_count)*0.5;
        runRightMotor(speed);
        delay(5);
    }

    //stop
    analogWrite(R_SPD, 1);
}

void turnRight(int degree)
{
    int distance = round(((float) degree*2*PI*145)/(0.1567*360));
    setupMotorsForward();
    //acceleration
    while (right_encoder_count<100)
    {
        int speed = right_encoder_count*0.5 + 10;
        runLeftMotor(speed);
        delay(10);
    }

    //motion
    int dist_temp = distance - 100;
    while (right_encoder_count<dist_temp)
    {
        runLeftMotor(50);
        delay(50);
    }

    //deseleration
    while (right_encoder_count<distance)
    {
        int speed = (distance - right_encoder_count)*0.5;
        runLeftMotor(speed);
        delay(5);
    }

    //stop
    analogWrite(R_SPD, 1);
}

//Functions for turning robot on spot
void turnOnSpot(int distance)
{
    int avarage = (abs(right_encoder_count)+abs(left_encoder_count))/2;
    //acceleration
    while (avarage<100)
    {
        int speed = avarage*0.5 + 10;
        turn(speed);
        delay(10);
        avarage = (abs(right_encoder_count)+abs(left_encoder_count))/2;
    }

    //motion
    int dist_temp = distance - 100;
    while (avarage<dist_temp)
    {
        turn(50);
        delay(50);
        avarage = (abs(right_encoder_count)+abs(left_encoder_count))/2;
    }

    //deseleration
    while (avarage<distance)
    {
        int speed = (distance - dist_temp)*0.5;
        turn(speed);
        delay(10);
        avarage = (abs(right_encoder_count)+abs(left_encoder_count))/2;
        if (avarage > distance-2)
        {
            break;
        }
        //Serial.println(avarage);
    }
    //stop
    analogWrite(R_SPD, 0);
    analogWrite(L_SPD, 0);

    Serial.println("Done2!!!");

    delay(25);


}

void turnLeftonSpot(float degree)
{
    int distance = round((degree*PI*145)/(0.1572*360));

    left_encoder_count=0;
    right_encoder_count=0;
    digitalWrite(L_DIR, HIGH);
    digitalWrite(R_DIR, LOW);

    turnOnSpot(distance);
}

void turnRightonSpot(float degree)
{
    int distance = round((degree*PI*145)/(0.1575*360));

    left_encoder_count=0;
    right_encoder_count=0;
    digitalWrite(L_DIR, LOW);
    digitalWrite(R_DIR, HIGH);

    turnOnSpot(distance);
}

void turnAnyAngle(float angle)
{
    if (angle>0)
    {
        turnLeftonSpot(angle);
    } else{
        turnRightonSpot(-angle);
    }
}

