/***************************************************
 *  This is a library for control of a DC motor via a TLC Led driver with angle
 *  sensor feedback on an ESP32.
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/
#include "MotorUnit.h"
#include "ArmLogger.h"
#include "math.h"
/*!
 *  @brief  Instantiates a new MotorUnit class. Instantiates classes for
 *          all other necessary controls.
 *  @param  tlc Pointer to a TLC59711 object to output PWM signals
 *  @param  forwardPin Output pin number for the TLC59711. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the TLC59711. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current readback
 *  @param  senseResistor Value in Ohms of the sense resistor for this channel
 *  @param  cal ESP32 adc calibration results for more accurate readings
 *  @param  angleCS ESP32 pin for the chip select of the angle sensor
 *
 */
MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmperrev){
    motor.reset(new DRV8873LED(tlc, forwardPin, backwardPin, readbackPin, senseResistor, cal));
    angleSensor.reset(new AS5048A(angleCS));
    angleSensor->init();
}

/*!
 *  @brief  Set a new max speed setting for the cycle
 *  @param newSpeed New speed from {SLOW, MEDIUMSLOW, MEDIUMFAST, FAST}
 */
void MotorUnit::setSpeed(speed newSpeed){
    currentSpeedSetting = newSpeed;
    switch (currentSpeedSetting) {
        case SLOW:
            currentSpeed = 35000;
            printMessage("Running at slow speed");
            break;
        case MEDIUMSLOW:
            currentSpeed = 45000;
            printMessage("Running at medium slow speed");
            break;
        case MEDIUMFAST:
            currentSpeed = 52000;
            printMessage("Running at medium fast speed");
            break;
        case FAST:
            currentSpeed = 60000;
            printMessage("Running at fast speed");
            break;
    }
}

/*!
 *  @brief  Return the current max speed setting for the cycle
 */
speed MotorUnit::getSpeed(){
    return currentSpeedSetting;
}

/*!
 *  @brief  Set the minimum movement angle to the currently measured angle
 */
void MotorUnit::setMinAngle(){
    minAngle = getControllerState();
}

/*!
 *  @brief  Get the minimum movement angle set during calibration
 */
float MotorUnit::getMinAngle(){
    return minAngle;
}/*!
 *  @brief  Set the minimum movement angle to the currently measured angle
 */
void MotorUnit::setMaxAngle(){
    maxAngle = getControllerState();
}

/*!
 *  @brief  Get the minimum movement angle set during calibration
 */
float MotorUnit::getMaxAngle(){
    return maxAngle;
}

/*!
 *  @brief  Set the current movement direction the flexion
 */
void   MotorUnit::setFlexion(){
    currentDirection = FLEXION;
}

/*!
 *  @brief  Set the current movement direction the extension
 */
void   MotorUnit::setExtension(){
    currentDirection = EXTENSION;
}

/*!
 *  @brief  Get the currently set movement direction
 */
dir    MotorUnit::getDirection(){
    return currentDirection;
}

/*!
 *  @brief  Set the active control mode
 *  @param newMode The enum member of the desired mode
 */
void MotorUnit::setControlMode(mode newMode){
    controlMode = newMode;
    stop();
}

/*!
 *  @brief  Retrive the active control mode
 *  @return The enum member of the current mode
 */
mode MotorUnit::getControlMode(){
    return controlMode;
}

/*!
 *  @brief  Compute the necessary output to achieve the desired motion and
 *  command the motor to that output
 */
void MotorUnit::computeSpeed(){
    lastInterval = (millis() - lastUpdate)/1000.0;
    lastUpdate = millis();
    currentState = getControllerState();
    // Normalize angle to between 0-2*pi, then take the cosine to get smooth speed curve
    // Scale speed to max speed
    outputMagnitude = currentSpeed;//35000;//((-cos(((currentState-minAngle)/maxAngle)*PI*2) + 1)/2 * (currentSpeed - minSpeed)) + minSpeed;
    if (currentDirection == FLEXION) {
        output = outputMagnitude;
    } else {
        output = -outputMagnitude;
    }
    if(disabled == false){
        motor->runAtPID(output);
    }else{
        motor->stop();
    }
}

/*!
 *  @brief  Retrieve the current state of the motor appropriate for the currently
 *  set control mode.
 *  @return Actual state of the motor in degrees or degrees/s
 */
float MotorUnit::getControllerState(){
    anglePrevious = angleCurrent;
    angleCurrent = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    if(controlMode == ANGLE){
        return angleCurrent;
    }else{
        return ((angleCurrent-anglePrevious)/lastInterval);
    }
}

/*!
 *  @brief  Retrieve the most receent angle measurement without triggering a new measurement
 *  @return Angle of the motor in degrees
 */
float MotorUnit::getCurrentAngle(){
    return angleCurrent;
}

/*!
 *  @brief Stop the motor immediately. Must be reset after stopping
 */
void MotorUnit::stop(){
    _disableControl();
    motor->stop();
}

/*!
 *  @brief  Resets the motor after a stop. Makes sure the motor does
 *  not move after being re-enabled
 */
void MotorUnit::reset(){
    _enableControl();
}

/*!
 *  @brief  Disables PID loop (loop still runs, but always commands motor to stop)
 */
void MotorUnit::_disableControl(){
    disabled = true;
}

/*!
 *  @brief  Enables PID loop (output will now be sent to motors)
 */
void MotorUnit::_enableControl(){
    disabled = false;
}
