#ifndef Commands_h
#define Commands_h
#include <Arduino.h>
#include <stdio.h>

unsigned long logTime = millis();
enum logType {DATA, LOG};


void printMessage(String message){
    char buff[80];
    sprintf(buff, "%s", message);
    outputLog(LOG, buff);
    return;
}

void printMessageFloat(String message, float value){
    char buff[80];
    sprintf(buff, "%s: %f", message, value);
    outputLog(LOG, buff);
    return;
}

void printMessageHex(String message, int value){
    char buff[80];
    sprintf(buff, "%s: %x", message, value);
    outputLog(LOG, buff);
    return;
}

void printMessageInt(String message, int value){
    char buff[80];
    sprintf(buff, "%s: %i", message, value);
    outputLog(LOG, buff);
    return;
}

void valueLog(float angle){
    char buff[80];
    sprintf(buff, "%f", angle);
    outputLog(DATA, buff);
    return;
}

String outputLog(logType l, char* message){
    logTime = micros();
    Serial.printf("%lu;%i;%s\n", logTime, l, message);
    return;
}

#endif