#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <cmath>

using namespace std;
using namespace std::chrono;

#define PIN1 7
#define PIN2 15

//measure distance function variables 
int prevState1 = 0; 
int currState1; 
int prevState2 = 0; 
int currState2; 
float stateCount1; 
float stateCount2; 
//radius of 2 wheels combined
float wheelSize = 62.5;
float wheelEncoderRes = 10;
float lengthBetweenWheels = 150;
bool timerCheck = false;
int moveTime = 1000;

double deg2rad(double deg)
{
    return deg * M_PI / 180;
}

void stopMotors(){
    cout << "Stopping!" << endl;
    digitalWrite(16,LOW);
    digitalWrite(0,LOW);    
    digitalWrite(7,LOW);
    digitalWrite(15,LOW);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    digitalWrite(1,LOW);
    digitalWrite(2,LOW);
}

void InitPins(){
    wiringPiSetup();
    cout << "Init!";
    //Front Right Motor
    pinMode(7,OUTPUT);
    pinMode(15,OUTPUT);
    //Front Left Motor
    pinMode(16,OUTPUT);
    pinMode(0,OUTPUT);
    //Back Right Motor
    pinMode(1,OUTPUT);
    pinMode(2,OUTPUT);
    //Back Left Motor
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);

    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);
    pinMode(14,OUTPUT);
    pinMode(6,OUTPUT);
    
    pinMode(29,INPUT);
    pinMode(28,INPUT);
    
    softPwmCreate(12,0,255);
    softPwmCreate(13,0,255);
    softPwmCreate(14,0,255);
    softPwmCreate(6,0,255);
    
    stopMotors();
}

//void setMotorSpeedsPWM(int16_t _motorFL, int16_t _motorFR, int16_t _motorBL, int16_t _motorBR)
//{
    //cout << "Set!"<< endl;
    //// Speed < 0 = backwards Speed > 0 forwards
    ////get the target direction requested
    //if (_motorFL < 0){
        //digitalWrite(16,LOW);
        //digitalWrite(0,HIGH);
    //}
    //else if(_motorFL == 0){
        //digitalWrite(16,LOW);
        //digitalWrite(0,LOW);
    //}
    //else {
        //digitalWrite(16,HIGH);
        //digitalWrite(0,LOW);
    //}

    //if (_motorFR < 0){
        //digitalWrite(7,LOW);
        //digitalWrite(15,HIGH);
    //}
    //else if(_motorFR == 0){
        //digitalWrite(7,LOW);
        //digitalWrite(15,LOW);
    //}
    //else {
        //digitalWrite(7,HIGH);
        //digitalWrite(15,LOW);
    //}

    //if (_motorBL < 0){
        //digitalWrite(3,LOW);
        //digitalWrite(4,HIGH);
    //}
    //else if(_motorBL == 0){
        //digitalWrite(3,LOW);
        //digitalWrite(4,LOW);
    //}
    //else {
        //digitalWrite(3,HIGH);
        //digitalWrite(4,LOW);
    //}

    //if (_motorBR < 0){
        //digitalWrite(1,LOW);
        //digitalWrite(2,HIGH);
    //}
    //else if(_motorBR == 0){
        //digitalWrite(1,LOW);
        //digitalWrite(2,LOW);
    //}
    //else {
        //digitalWrite(1,HIGH);
        //digitalWrite(2,LOW);
    //}

    ////set speed to invididual pins here.
    //softPwmWrite(12, abs(_motorFL));
    //softPwmWrite(13, abs(_motorFR));
    //softPwmWrite(14, abs(_motorBL));
    //softPwmWrite(6, abs(_motorBR));
//}

void setMotorSpeedsPWM(int16_t _motorL, int16_t _motorR)
{
    cout << "Set!"<< endl;
    // Speed < 0 = backwards Speed > 0 forwards
    //get the target direction requested
    if (_motorL < 0){
        digitalWrite(16,LOW);
        digitalWrite(0,HIGH);
        digitalWrite(3,LOW);
        digitalWrite(4,HIGH);
    }
    else if(_motorL == 0){
        digitalWrite(16,LOW);
        digitalWrite(0,LOW);
        digitalWrite(3,LOW);
        digitalWrite(4,LOW);
    }
    else {
        digitalWrite(16,HIGH);
        digitalWrite(0,LOW);
        digitalWrite(3,HIGH);
        digitalWrite(4,LOW);
    }
    if (_motorR < 0){
        digitalWrite(7,LOW);
        digitalWrite(15,HIGH);
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
    }
    else if(_motorR == 0){
        digitalWrite(7,LOW);
        digitalWrite(15,LOW);
        digitalWrite(1,LOW);
        digitalWrite(2,LOW);
    }
    else {
        digitalWrite(7,HIGH);
        digitalWrite(15,LOW);
        digitalWrite(1,HIGH);
        digitalWrite(2,LOW);
    }
    
    softPwmWrite(12, abs(_motorL));
    softPwmWrite(13, abs(_motorR));
    softPwmWrite(14, abs(_motorL));
    softPwmWrite(6, abs(_motorR));
}

float measureDist(){
    //time start
    float distance1 = 99999;
    float distance2 = 99999;
    int timertemp = 0;
    auto timerStart = steady_clock::now(); 
    while(timerCheck != true){
        currState1 = digitalRead(29);
        currState2 = digitalRead(28);
        if(currState1 != prevState1){
            prevState1 = currState1;
            stateCount1 += 1;
        }
        if(currState2 != prevState2){
            prevState2 = currState2;
            stateCount2 += 1;
        }
        
        duration<double, std::milli> timer = (steady_clock::now() - timerStart); 
        //timertemp = duration_cast<milliseconds>(timer).count();
        //cout << timertemp << endl;
        if(duration_cast<milliseconds>(timer).count() == moveTime){
            timerCheck = true;
        }
        
    }
    
    distance1 = (stateCount1 / wheelEncoderRes) * wheelSize;
    distance2 = (stateCount2 / wheelEncoderRes) * wheelSize;
    cout << stateCount1 << endl;
    cout << stateCount2 << endl;
    cout << distance1 << endl;
    cout << distance2 << endl;
    
    
    cout << "distance1 - " << distance1 << "mm velocity - " << (distance1/moveTime)*1000 << "mm/s" << endl;
    cout << "distance2 - " << distance2 << "mm velocity - " << (distance2/moveTime)*1000 << "mm/s" << endl;
    stateCount1 = 0; 
    stateCount2 = 0; 
    timerCheck = false;
    return distance1;
}

int velocityToPWM(float input){
    //max velocity input 110mm/s
    int val;
    if(input >0){
        val = round(2.00689 * abs(input) + 14.41695);
    }
    else{
        val = -(round(2.00689 * abs(input) + 14.41695));
    }
    cout<<val<<endl;
    return val;
}



float* calVelocity(float inputVelocity,float inputAngle){
    float* velocity = new float[2];
    
    velocity[0] = (2*inputVelocity + deg2rad(inputAngle)*lengthBetweenWheels)/(2*wheelSize);
    velocity[1] = (2*inputVelocity - deg2rad(inputAngle)*lengthBetweenWheels)/(2*wheelSize);
    return velocity;
} 

int main(int argc, char const *argv[]){
    wiringPiSetup();
    InitPins();
    
    
    float* velocity = calVelocity(0,270);
    cout << velocity[0] << " " << velocity[1]<< endl;
    
    int speed1 = velocityToPWM(velocity[0]);
    int speed2 = velocityToPWM(velocity[1]);
    
    
    setMotorSpeedsPWM(98,-98);
    cout << measureDist() << endl;
    stopMotors();
}
//int main(int argc, char const *argv[]){
    //wiringPiSetup();
    //cout << "Running!" << endl;
    //InitPins();
    //while(1){
        ////forward
        //setMotorSpeedsPWM(255,255,255,255);
        //delay(5000);
        //stopMotors();
        //delay(1000);
        ////backward
        //setMotorSpeedsPWM(-255,-255,-255,-255);
        //delay(5000);
        //stopMotors();
        //delay(1000);
        ////turn left
        //setMotorSpeedsPWM(0,0,0,255);
        //delay(5000);
        //stopMotors();
        //delay(1000);
        ////turn right
        //setMotorSpeedsPWM(0,0,255,0);
        //delay(5000);
        //stopMotors();
        //delay(1000);
        ////speed control 
        //setMotorSpeedsPWM(255,-128,-64,32);
        //delay(5000);
        //stopMotors();
        //delay(1000);
    //}
    //return 0;
//}

 //int main(int argc, char const *argv[])
 //{
     //wiringPiSetup();

     //pinMode(7,OUTPUT);
     //pinMode(15,OUTPUT);
     //pinMode(16,OUTPUT);
     //pinMode(0, INPUT);
    
     //digitalWrite(7,HIGH);
     //digitalWrite(15,LOW);
     //digitalWrite(16,LOW);
    
     //cout << "reading";
     //cout << digitalRead(7);
     //cout << digitalRead(15);
     //cout << digitalRead(16);

     //cout << "writing";
     //digitalWrite(7,HIGH);
     //digitalWrite(15,LOW);
     //digitalWrite(16,LOW);
    
     //cout << "reading";
     //cout << digitalRead(7);
     //cout << digitalRead(15);
     //cout << digitalRead(16);
    
     ////while(1){
         ////cout<< digitalRead(0);
     ////}
     //cout << "Hello World!";
     //return 0;
 //}
