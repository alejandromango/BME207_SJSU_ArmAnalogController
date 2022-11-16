/*!
 *  @file DRV8873LED.h
 *
 *  This is a library to interact with the TI DRV8873 chip via a peripheral PWM generator chip (TLC59711)
 *
 */

#ifndef DRV8873LED_H
#define DRV8873LED_H

enum direction {BACKWARD, FORWARD};

#include <Arduino.h>
#include "TLC59711.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          DRV8873 Sensor
 */
class DRV8873LED{
public:
    DRV8873LED(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal);


    void forward(uint16_t speed);
    void fullForward();
    void backward(uint16_t speed);
    void fullBackward();
    void runAtSpeed(uint8_t direction, uint16_t speed);
    void runAtPID(int signed_speed);
    void stop();
    void highZ();
    double readCurrent();

private:
    int multisamples = 100;
    TLC59711 *_driver;
    uint8_t _forward, _back;
    adc1_channel_t _readback;
    double _rsense;
    esp_adc_cal_characteristics_t *_cal_values;
};

#endif
