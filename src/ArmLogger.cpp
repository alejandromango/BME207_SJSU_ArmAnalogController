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
    int mes_len = message.length() + 1;
    char mes[mes_len];
    message.toCharArray(mes, mes_len);
    sprintf(buff, "%s", mes);
    outputLog(LOG, buff);
}

void printMessageFloat(String message, float value){
    char buff[80];
    int mes_len = message.length() + 1;
    char mes[mes_len];
    message.toCharArray(mes, mes_len);
    sprintf(buff, "%s: %f", mes, value);
    outputLog(LOG, buff);
}

void printMessageHex(String message, int value){
    char buff[80];
    int mes_len = message.length() + 1;
    char mes[mes_len];
    message.toCharArray(mes, mes_len);
    sprintf(buff, "%s: %x", mes, value);
    outputLog(LOG, buff);
}

void printMessageInt(String message, int value){
    char buff[80];
    int mes_len = message.length() + 1;
    char mes[mes_len];
    message.toCharArray(mes, mes_len);
    sprintf(buff, "%s: %i", mes, value);
    outputLog(LOG, buff);
}

void angleLog(float angle){
    char buff[80];
    sprintf(buff, "%f", angle);
    outputLog(DATA, buff);
}