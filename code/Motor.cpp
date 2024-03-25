#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

#define PIN1 7
#define PIN2 15

//measure distance function variables 
int prevState = 0; 
int currState; 
float stateCount; 
float wheelSize;
float distance;
float wheelEncoderRes = 10;
auto timer;




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
    
    softPwmCreate(12,0,255);
    softPwmCreate(13,0,255);
    softPwmCreate(14,0,255);
    softPwmCreate(6,0,255);
    
    stopMotors();
}

void setMotorSpeedsPWM(int16_t _motorFL, int16_t _motorFR, int16_t _motorBL, int16_t _motorBR)
{
    cout << "Set!"<< endl;
    // Speed < 0 = backwards Speed > 0 forwards
    //get the target direction requested
    if (_motorFL < 0){
        digitalWrite(16,LOW);
        digitalWrite(0,HIGH);
    }
    else if(_motorFL == 0){
        digitalWrite(16,LOW);
        digitalWrite(0,LOW);
    }
    else {
        digitalWrite(16,HIGH);
        digitalWrite(0,LOW);
    }

    if (_motorFR < 0){
        digitalWrite(7,LOW);
        digitalWrite(15,HIGH);
    }
    else if(_motorFR == 0){
        digitalWrite(7,LOW);
        digitalWrite(15,LOW);
    }
    else {
        digitalWrite(7,HIGH);
        digitalWrite(15,LOW);
    }

    if (_motorBL < 0){
        digitalWrite(3,LOW);
        digitalWrite(4,HIGH);
    }
    else if(_motorBL == 0){
        digitalWrite(3,LOW);
        digitalWrite(4,LOW);
    }
    else {
        digitalWrite(3,HIGH);
        digitalWrite(4,LOW);
    }

    if (_motorBR < 0){
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
    }
    else if(_motorBR == 0){
        digitalWrite(1,LOW);
        digitalWrite(2,LOW);
    }
    else {
        digitalWrite(1,HIGH);
        digitalWrite(2,LOW);
    }

    //set speed to invididual pins here.
    softPwmWrite(12, abs(_motorFL));
    softPwmWrite(13, abs(_motorFR));
    softPwmWrite(14, abs(_motorBL));
    softPwmWrite(6, abs(_motorBR));
}

//void measureDist(){
    ////time start
    //auto timerStart = steady_clock::now(); 
    //while((//something != 1000){
        //currState = digitalRead(29);
        //if(currState != prevState){
            //prevState = currState;
            //stateCount += 1;
        //}
        
        //timer = 
    //}
    
    //distance = (stateCount / wheelEncoderRes) * wheelSize;
    //stateCount = 0; 
    //return distance;
}

int main(int argc, char const *argv[]){
    wiringPiSetup();
    InitPins();
    setMotorSpeedsPWM(255,255,255,255);
    while(1){
        cout << digitalRead(29) << endl;
    }
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
