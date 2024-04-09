#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <chrono>
#include <cmath>
#include "PID_v2.cpp"

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
////radius of 2 wheels combined
float wheelSize = 62.5;
float wheelEncoderRes = 10;
float lengthBetweenWheels = 150;
bool timerCheck = false;
int moveTime = 250;

//state of robot variables 
//varaibles used to describe the current state of the robot in relation to the world view
double currentAngle = 0;
double currentXPos = 0;
double currentYPos = 0;

double errorRoom = 25;

//PID
//forwards = 1 backwards = -1
int wheelDirR = 1;
int wheelDirL = 1;
double wheelVelR = 0;
double wheelVelL = 0;
double currentWheelVelR = 0;
double currentWheelVelL = 0;
double targetWheelVelR = 0;
double targetWheelVelL = 0;
double wheelPWML = 0;
double wheelPWMR = 0;
double* velocities;

//wheel PID settings
float kp = 0.5;
float ki = 1;
float kd = 0;

//movement PID settings
float mkp = 0.5;
float mki = 0;
float mkd = 0;
double travelTime = 1000;
double maxVelocity = 110;
double targetAngle = 0;
double targetXPos = 0;
double targetYPos = 0;
double instAngle = 0;
double instXPos = 0;
double instYPos = 0;

PID leftMotorPID(&currentWheelVelL,&wheelPWML,&targetWheelVelL,kp,ki,kd,DIRECT);
PID rightMotorPID(&currentWheelVelR,&wheelPWMR,&targetWheelVelR,kp,ki,kd,DIRECT);

PID movemnentXPID(&currentXPos,&instXPos,&targetXPos,kp,ki,kd,DIRECT);
PID movemnentYPID(&currentYPos,&instYPos,&targetYPos,kp,ki,kd,DIRECT);
PID movemnentAPID(&currentAngle,&instAngle,&targetAngle,kp,ki,kd,DIRECT);

void PIDinit(){
    leftMotorPID.SetMode(AUTOMATIC);
    leftMotorPID.SetSampleTime(moveTime);
    leftMotorPID.SetOutputLimits(0, 255);
    
    rightMotorPID.SetMode(AUTOMATIC);
    rightMotorPID.SetSampleTime(moveTime);
    rightMotorPID.SetOutputLimits(0, 255);
    
    movemnentXPID.SetMode(AUTOMATIC);
    movemnentXPID.SetSampleTime(travelTime);
    movemnentXPID.SetOutputLimits(0, maxVelocity);
    
    movemnentYPID.SetMode(AUTOMATIC);
    movemnentYPID.SetSampleTime(travelTime);
    movemnentYPID.SetOutputLimits(0, maxVelocity);
    
    movemnentAPID.SetMode(AUTOMATIC);
    movemnentAPID.SetSampleTime(travelTime);
    movemnentAPID.SetOutputLimits(0, 360);
}

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
    
    wheelPWML = _motorL;
    wheelPWMR = _motorR;
}

void updateMotorPWM(){
    cout << "update!"<< endl;
    // Speed < 0 = backwards Speed > 0 forwards
    //get the target direction requested
    if (wheelDirL < 0){
        digitalWrite(16,LOW);
        digitalWrite(0,HIGH);
        digitalWrite(3,LOW);
        digitalWrite(4,HIGH);
    }
    else if(wheelDirL == 0){
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
    if (wheelDirR < 0){
        digitalWrite(7,LOW);
        digitalWrite(15,HIGH);
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
    }
    else if(wheelDirR == 0){
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
    
    softPwmWrite(12, abs(wheelPWML));
    softPwmWrite(13, abs(wheelPWMR));
    softPwmWrite(14, abs(wheelPWML));
    softPwmWrite(6, abs(wheelPWMR));

}

float* measureDist(){
    //time start
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
    
    float* distance = new float[2];
    distance[0] = (stateCount1 / wheelEncoderRes) * wheelSize;
    distance[1] = (stateCount2 / wheelEncoderRes) * wheelSize;
    //cout << stateCount1 << endl;
    //cout << stateCount2 << endl;
    //cout << distance1 << endl;
    //cout << distance2 << endl;
    
    
    //cout << "distance1 - " << distance1 << "mm velocity - " << (distance1/moveTime)*1000 << "mm/s" << endl;
    //cout << "distance2 - " << distance2 << "mm velocity - " << (distance2/moveTime)*1000 << "mm/s" << endl;
    stateCount1 = 0; 
    stateCount2 = 0; 
    timerCheck = false;
    
    
    return distance;
}

//cal Xdot Ydot and angleDot
void calInstantaneousMovement(){
    movemnentXPID.Compute();
    movemnentYPID.Compute();
    movemnentAPID.Compute();
}

void setTargetLocation(double xGoal, double yGoal, double angleGoal){
    targetAngle = angleGoal;
    targetXPos = xGoal;
    targetYPos = yGoal;
}

void updateCurrLocation(double rwheeldist, double lwheeldist){
    double centreDist = (rwheeldist + lwheeldist) /2;
    currentXPos = currentXPos + (centreDist * cos(currentAngle));
    currentYPos = currentYPos + (centreDist * sin(currentAngle));
    currentAngle = currentAngle + ((rwheeldist - lwheeldist )/ lengthBetweenWheels);
}

double* measureVel(){
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
    //cout << distance1 << endl;
    //cout << distance2 << endl;
    
    double* velocity = new double[2];
    
    velocity[0] = distance1/moveTime*1000;
    velocity[1] = distance2/moveTime*1000;
    //cout << velocity[0] << endl;
    //cout << velocity[1] << endl;
    stateCount1 = 0; 
    stateCount2 = 0; 
    timerCheck = false;
    return velocity;
}

//cal individual wheel vels
float* calWheelVel(double xDot, double yDot, double angleDot){
    float* wheelVel = new float[2];
    //left wheel
    wheelVel[0] = xDot*cos(currentAngle)+yDot*sin(currentAngle) - ((angleDot * lengthBetweenWheels)/2*wheelSize);
    //right wheel 
    wheelVel[1] = xDot*cos(currentAngle)+yDot*sin(currentAngle) + ((angleDot * lengthBetweenWheels)/2*wheelSize);
    return wheelVel;
}

//not needed?
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

//sets the target velocity for each wheel for the PID to use
void setTargetVelocity(float leftTarget, float rightTarget){
    if(leftTarget < 0){
        wheelDirL = -1;
    }
    else{
        wheelDirL = 1; 
    }
    if(rightTarget < 0){
        wheelDirR = -1;
    }
    else{
        wheelDirR = 1; 
    }
    
    targetWheelVelR = abs(rightTarget);
    targetWheelVelL = abs(leftTarget);
}

//updates the PID for each wheel 
void updatePID(){
    if(leftMotorPID.Compute()){
        currentWheelVelL = 0; 
        //cout<<"update left" << endl;
    }
    if(rightMotorPID.Compute()){
        currentWheelVelR = 0; 
    }
}

//loop to update the speed of each wheel 
void loop(){
    velocities = measureVel();
    currentWheelVelL = velocities[0]; 
    currentWheelVelR = velocities[1]; 
    cout << "Left Velocity:"<< velocities[0]*wheelDirL << " Right Velocity: " << velocities[1]*wheelDirR << endl;
    
    updatePID();
    
    updateMotorPWM();
}

int main(int argc, char const *argv[]){
    wiringPiSetup();
    InitPins();
    PIDinit();
    setTargetVelocity(80,-80);
    setMotorSpeedsPWM(0,0);
    cout << "start" << endl;
    while(1){
        loop();
        //cout << "target" <<targetWheelVelL << endl;
    }
    stopMotors();
}

//int main(int argc, char const *argv[]){
    //wiringPiSetup();
    //InitPins();
    
    
    //float* velocity = calVelocity(0,270);
    //cout << velocity[0] << " " << velocity[1]<< endl;
    
    
    
    //setMotorSpeedsPWM(98,-98);
    //cout << measureDist() << endl;
    //stopMotors();
//}
