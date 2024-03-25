#include <iostream>
#include <wiringPi.h>
using namespace std;

#define PIN1 7
#define PIN2 15

void stopMotors(){
    cout << "Stopping!";
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

    //pinMode(12,PWM_OUTPUT);
    //pinMode(13,PWM_OUTPUT);
    //pinMode(14,PWM_OUTPUT);
    //pinMode(6,PWM_OUTPUT);
    
    stopMotors();
}

void setMotorSpeedsPWM(int16_t _motorFL, int16_t _motorFR, int16_t _motorBL, int16_t _motorBR)
{
    cout << "Set!";
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
    //pwmWrite(12, _motorFL * 4);
    //pwmWrite(13, _motorFR * 4);
    //pwmWrite(14, _motorBL * 4);
    //pwmWrite(6, _motorBR * 4);
}


int main(int argc, char const *argv[]){
    wiringPiSetup();
    cout << "Running!";
    InitPins();
    while(1){
        //forward
        setMotorSpeedsPWM(255,255,255,255);
        delay(5000);
        stopMotors();
        delay(1000);
        //backward
        setMotorSpeedsPWM(-255,-255,-255,-255);
        delay(5000);
        stopMotors();
        delay(1000);
        //turn left
        setMotorSpeedsPWM(0,0,0,255);
        delay(5000);
        stopMotors();
        delay(1000);
        //turn right
        setMotorSpeedsPWM(0,0,255,0);
        delay(5000);
        stopMotors();
        delay(1000);
        ////speed control 
        //setMotorSpeedsPWM(255,-128,-64,32);
        //delay(5000);
        //stopMotors();
        //delay(1000);
    }
    return 0;
}

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
