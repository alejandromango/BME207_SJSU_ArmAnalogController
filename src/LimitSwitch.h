/*!
 *  @file LimitSwitch.h
 *
 *  This is a two-wire library for the TI LimitSwitch chip
 *
 *  Two pins are required to send data: clock and data pin.
 *
 *  Code based on Adafruit_LimitSwitch by Limor Fried/Ladyada (Adafruit Industries).
 *
 */

#ifndef LimitSwitch_H
#define LimitSwitch_H

#include <Arduino.h>

/*!
 *  @brief  Class that stores the information for an individual limit switch
 */
class LimitSwitch{
public:
    LimitSwitch(uint8_t pinNum, bool pullup);
    int getState();
    enum SwitchState {ON_LIMIT, OFF_LIMIT};
private:
    int switchValue;
    int8_t _pin;
    bool _pullup;
    SwitchState state;
};

#endif
