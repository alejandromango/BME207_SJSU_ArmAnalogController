/***************************************************
 *  Read and track the on/off state of a limit switch
 * 
 *  Takes a single GPIO pin as input
 *
 *  By Alexander Martin-Ginnold
 ****************************************************/

#include "LimitSwitch.h"

/*!
 *  @brief  Instantiates a new LimitSwitch class for generic two-wire control
 *  @param  pinNum
 *          number of connected drivers
 *  @param  pullup
 *          Should the pin be configured pullup (true) or pulldown (false)
 */
LimitSwitch::LimitSwitch(gpio_num_t pinNum, bool pullup){
    _pin = pinNum;
    _pullup = pullup;
    if (_pullup)
        pinMode(_pin, INPUT_PULLUP);

    else
        pinMode(_pin, INPUT_PULLDOWN);
}

/*!
 *  @brief  Read the limit switch and return whether the limit has been hit. Assumes a normally closed switch
 *  
 */
int LimitSwitch::getState(){
    switchValue = digitalRead(_pin);
    if (switchValue == LOW){
        state = OFF_LIMIT;
    }else{
        state = ON_LIMIT;
    }   
    
    return state;
}