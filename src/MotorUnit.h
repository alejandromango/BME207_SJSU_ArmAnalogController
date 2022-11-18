#ifndef MotorUnit_h
#define MotorUnit_h
#include "memory"

#include <Arduino.h>
#include "TLC59711.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "DRV8873LED.h"
#include "AS5048A.h"

enum mode {ANGLE, SPEED};
enum speed {SLOW, MEDIUMSLOW, MEDIUMFAST, FAST};
enum dir {FLEXION, EXTENSION};

class MotorUnit{

public:
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmperrev);
    std::unique_ptr<MiniPID> pid;
    std::unique_ptr<DRV8873LED> motor;
    std::unique_ptr<AS5048A> angleSensor;
    void   setSpeed(speed s);
    speed  getSpeed();
    void   setMinAngle();
    float  getMinAngle();
    void   setMaxAngle();
    float  getMaxAngle();
    void   setFlexion();
    void   setExtension();
    dir    getDirection();
    void   setControlMode(mode newMode);
    mode   getControlMode();
    void   computeSpeed();
    float  getControllerState();
    void   stop();
    void   reset();

private:
    void   _disableControl();
    void   _enableControl();

    float lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    bool disabled = false;

    int output = 0;
    float currentState = 0.0;
    float angleCurrent  = 0.0;
    float minAngle = 0.0;
    float maxAngle = 0.0;
    int outputMagnitude = 0;
    float anglePrevious = 0.0;

    float mampsCurrent  = 0.0;
    mode controlMode = ANGLE;
    speed currentSpeedSetting = SLOW;
    dir currentDirection = EXTENSION;
    int currentSpeed = 40000;
    int minSpeed = 35000;

};

#endif
