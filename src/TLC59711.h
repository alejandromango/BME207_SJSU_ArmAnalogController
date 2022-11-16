/*!
 *  @file TLC59711.h
 *
 *  This is a two-wire library for the TI TLC59711 chip
 *
 *  Two pins are required to send data: clock and data pin.
 *
 *  Code based on Adafruit_TLC59711 by Limor Fried/Ladyada (Adafruit Industries).
 *
 */

#ifndef TLC59711_H
#define TLC59711_H

#include <Arduino.h>

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          TLC59711 Sensor
 */
class TLC59711{
public:
    TLC59711(uint8_t n, uint8_t c, uint8_t d);

    boolean begin();

    void setPWM(uint8_t chan, uint16_t pwm);
    void write();
    void writeMSB(uint8_t d);

private:
    uint16_t *pwmbuffer;

    uint8_t BCr, BCg, BCb;
    int8_t numdrivers, _clk, _dat, _Option;
};

#endif
