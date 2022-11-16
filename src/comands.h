#ifndef Commands_h
#define Commands_h

#include "MotorUnit.h"

float setPoint1 = 0;
float setPoint2 = 0;
float setPoint3 = 0;
float setPoint4 = 0;
float setPoint5 = 0;
bool setpointFlag = false;

float proportional = 100000;
float integral = 10;
float derivative = 0.0;
bool pidFlag = false;

mode updatedMode = REVOLUTIONS;
bool modeFlag = false;

void parseCommand(String command, String value){
    Serial.println("Parsing Command:");
    Serial.println(command);

    if(command == "setpoint1"){
        setPoint1 = value.toFloat();
        setpointFlag = true;
    }
    if(command == "setpoint2"){
        setPoint2 = value.toFloat();
        setpointFlag = true;
    }
    if(command == "setpoint3"){
        setPoint3 = value.toFloat();
        setpointFlag = true;
    }
    if(command == "setpoint4"){
        setPoint4 = value.toFloat();
        setpointFlag = true;
    }
    if(command == "setpoint5"){
        setPoint5 = value.toFloat();
        setpointFlag = true;
    }
    if(command == "setproportional"){
        proportional = value.toFloat();
        pidFlag = true;
    }
    if(command == "setintegral"){
        integral = value.toFloat();
        pidFlag = true;
    }
    if(command == "setderivative"){
        derivative = value.toFloat();
        pidFlag = true;
    }
    if(command == "setcontrolmode"){
        updatedMode = (mode)value.toInt();
        if(updatedMode > MAX){
            updatedMode = REVOLUTIONS;
        }
        Serial.print("New mode: ");
        Serial.printf("%i\n", updatedMode);
        modeFlag = true;
    }
}

#endif
