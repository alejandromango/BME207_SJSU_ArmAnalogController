/***************************************************
 *   This is a library to interact with the TI DRV8873 chip via a peripheral
 *   PWM generator chip (TLC59711)
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/

#include "DRV8873LED.h"

/*!
 *  @brief  Instantiates a new DRV8873LED class for generic two-wire control
 *  @param  tlc Pointer to a TLC59711 object to output PWM signals
 *  @param  forwardPin Output pin number for the TLC59711. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the TLC59711. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current readback
 *  @param  senseResistor Value in Ohms of the sense resistor for this channel
 *  @param  cal ESP32 adc calibration results for more accurate readings
 */
DRV8873LED::DRV8873LED(TLC59711 *tlc,
                       uint8_t forwardPin,
                       uint8_t backwardPin,
                       adc1_channel_t readbackPin,
                       double senseResistor,
                       esp_adc_cal_characteristics_t *cal){
    _driver = tlc;
    _forward = forwardPin;
    _back = backwardPin;
    _readback = readbackPin;
    _rsense = senseResistor;
    _cal_values = cal;

}

/*!
 *  @brief  Run the motors forward at the given speed
 *  @param speed The speed the motor should spin (0-65535)
 */
void DRV8873LED::forward(uint16_t speed){
    runAtSpeed(FORWARD, speed);
}

/*!
 *  @brief  Run the motors forward at max speed
 */
void DRV8873LED::fullForward(){
    runAtSpeed(FORWARD, 65535);
}

/*!
 *  @brief  Run the motors backward at the given speed
 *  @param speed The speed the motor should spin (0-65535)
 */
void DRV8873LED::backward(uint16_t speed){
    runAtSpeed(BACKWARD, speed);
}

/*!
 *  @brief  Run the motors backward at max speed
 */
void DRV8873LED::fullBackward(){
    runAtSpeed(BACKWARD, 65535);
}

/*!
 *  @brief  Run the motors at the given speed. Interpret sign as backward for
 *  negative and forward for positive
 *  @param speed The speed the motor should spin (-65535 to 65535)
 */
void DRV8873LED::runAtPID(int signed_speed){
    if(signed_speed < 0){
        runAtSpeed(BACKWARD, abs(signed_speed));
    }else{
        runAtSpeed(FORWARD, abs(signed_speed));
    }
}

/*!
 *  @brief  Run the motors in the given direction at the given speed. All other
 *  speed setting functions use this to actually write to the outputs
 *  @param  direction Direction backward (0) or forward (1, or ~0)
 *  @param speed The speed the motor should spin (0-65535)
 */
void DRV8873LED::runAtSpeed(uint8_t direction, uint16_t speed){
    if(direction == 0){
        _driver->setPWM(_forward, 65535);
        _driver->setPWM(_back, 65535 - speed);

    }else{
        _driver->setPWM(_back, 65535);
        _driver->setPWM(_forward, 65535 - speed);
    }
    _driver->write();
}

/*!
 *  @brief  Stop the motors in a braking state
 */
void DRV8873LED::stop(){
    _driver->setPWM(_forward, 65535);
    _driver->setPWM(_back, 65535);
    _driver->write();

}

/*!
 *  @brief  Stop the motors in a high-z state
 */
void DRV8873LED::highZ(){
    _driver->setPWM(_forward, 0);
    _driver->setPWM(_back, 0);
    _driver->write();

}

/*!
 *  @brief  Read the value from an ADC and calculate the current. Allows
 *  multisampling to smooth signal
 *  NOTE: ESP32 adcs are non-linear and have deadzones at top and bottom.
 *        This value bottoms out above 0mA!
 *  @return Calibrated reading of current in mA.
 *
 */
double DRV8873LED::readCurrent(){
    int adcReadback = 0;
    for(int i= 0; i < multisamples; i++){
        adcReadback += adc1_get_raw(_readback);
        delayMicroseconds(10);
    }
    adcReadback = adcReadback/multisamples;
    double cal_mV = esp_adc_cal_raw_to_voltage(adcReadback, _cal_values);
    double zero_mV = esp_adc_cal_raw_to_voltage(0, _cal_values);
    return ((cal_mV)/_rsense)*1100.0;
}
