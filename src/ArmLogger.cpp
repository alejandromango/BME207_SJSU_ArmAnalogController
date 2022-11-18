#include <Arduino.h>
#include <stdio.h>
#include "ArmLogger.h"

unsigned long logTime = millis();

void outputLog(logType l, char* message){
    logTime = micros();
    Serial.printf("%lu;%i;%s\n", logTime, l, message);
}

void printMessage(String message){
    char buff[80];
    sprintf(buff, "%s", message);
    outputLog(LOG, buff);
}

void printMessageFloat(String message, float value){
    char buff[80];
    sprintf(buff, "%s: %f", message, value);
    outputLog(LOG, buff);
}

void printMessageHex(String message, int value){
    char buff[80];
    sprintf(buff, "%s: %x", message, value);
    outputLog(LOG, buff);
}

void printMessageInt(String message, int value){
    char buff[80];
    sprintf(buff, "%s: %i", message, value);
    outputLog(LOG, buff);
}

void angleLog(float angle){
    char buff[80];
    sprintf(buff, "%f", angle);
    outputLog(DATA, buff);
}