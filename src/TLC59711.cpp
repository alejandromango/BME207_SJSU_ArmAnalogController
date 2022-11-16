/***************************************************
 *  This is a two-wire library for the TI TLC59711 chip
 *
 *  Two pins are required to send data: clock and data pin.
 *
 *  Code based on Adafruit_TLC59711 by Limor Fried/Ladyada (Adafruit Industries).
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/

#include "TLC59711.h"

/*!
 *  @brief  Instantiates a new TLC59711 class for generic two-wire control
 *  @param  n
 *          number of connected drivers
 *  @param  c
 *          clock pin
 *  @param  d
 *          data pin
 */
TLC59711::TLC59711(uint8_t n, uint8_t c, uint8_t d){
    _Option = 1;
    numdrivers = n;
    _clk = c;
    _dat = d;

    BCr = BCg = BCb = 0x7F; // default 100% global brigthness

    pwmbuffer = (uint16_t *)calloc(2, 12 * n);
}

/*!
 *  @brief  Write data at MSB
 *  @param  d
 *          data
 */
void TLC59711::writeMSB(uint8_t d){

    uint32_t b = 0x80;
    //  b <<= (bits-1);
    for (; b != 0; b >>= 1){
        digitalWrite(_clk, LOW);
        if (d & b)
            digitalWrite(_dat, HIGH);
        else
            digitalWrite(_dat, LOW);
        digitalWrite(_clk, HIGH);
        delayMicroseconds(1);
    }
}

    /*!
 *  @brief  Writes PWM buffer to board
 */
void TLC59711::write(){
    uint32_t command;

    // Magic word for write
    command = 0x25;

    command <<= 5;
    // OUTTMG = 1, EXTGCK = 0, TMGRST = 1, DSPRPT = 1, BLANK = 0 -> 0x16
    command |= 0x16;

    command <<= 7;
    command |= BCr;

    command <<= 7;
    command |= BCg;

    command <<= 7;
    command |= BCb;

    noInterrupts();
    for (uint8_t n = 0; n < numdrivers; n++){
        writeMSB(command >> 24);
        writeMSB(command >> 16);
        writeMSB(command >> 8);
        writeMSB(command);

        //12 channels per TLC59711
        for (int8_t c = 11; c >= 0; c--){
            //16 bits per channel, send MSB first
            writeMSB(pwmbuffer[n * 12 + c] >> 8);
            writeMSB(pwmbuffer[n * 12 + c]);
        }
    }

    delayMicroseconds(200);

    interrupts();
}

/*!
 *  @brief  Set PWM value on selected channel
 *  @param  chan
 *          one from 12 channel (per driver) so there is 12 * number of drivers
 *  @param  pwm
 *          pwm value
 */
void TLC59711::setPWM(uint8_t chan, uint16_t pwm){
    if (chan > 12 * numdrivers)
        return;
    pwmbuffer[chan] = pwm;
}

/*!
 *  @brief  Begins connection if there is not empty PWM buffer
 *  @return If successful returns true, otherwise false
 */
boolean TLC59711::begin(){
    if (!pwmbuffer)
        return false;


    pinMode(_clk, OUTPUT);
    pinMode(_dat, OUTPUT);

    return true;
}
