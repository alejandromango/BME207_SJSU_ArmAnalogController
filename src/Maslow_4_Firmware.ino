#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "html.h"
#include "TLC59711.h"
#include "MotorUnit.h"
#include "LimitSwitch.h"
#include "Ticker.h"
#include "ArmLogger.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>

// #define BOARD_BRINGUP

unsigned long ourTime = millis();
bool hitFlexionLimit = false;
bool calibrationFinished = false;
bool needSpeed = true;
int cycleNumber = -1;
int numReplicates = 1;
const int numCycles = numReplicates * (FAST + 1);
enum cyclingStates {INIT, SETTLING, FLEXING, EXTENDING, FINISHED, ABORTED, RESETTING, NEXT};
cyclingStates cState = INIT;

// generate a randomly ordered array of speeds
// https://cplusplus.com/reference/algorithm/shuffle/
std::array<speed,4> cycleSpeeds {SLOW, MEDIUMSLOW, MEDIUMFAST, FAST};
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST,
                                //   SLOW, MEDIUMSLOW, MEDIUMFAST, FAST};
// obtain a time-based seed:
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_2_5);//ADC1_GPIO33_CHANNEL
esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_2_5);//ADC1_GPIO34_CHANNEL
esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_2_5);//ADC1_GPIO36_CHANNEL
esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_2_5);//ADC1_GPIO32_CHANNEL
esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_2_5);//ADC1_GPIO35_CHANNEL
#define NUM_TLC59711 1
#define tlcData   5
#define tlcClock  21
TLC59711 tlc = TLC59711(NUM_TLC59711, tlcClock, tlcData);
MotorUnit motor1 = MotorUnit(&tlc, 1, 0, ADC1_CHANNEL_5, 10000.0, adc_1_characterisitics, 17, 29);//ADC1_GPIO33_CHANNEL
MotorUnit motor2 = MotorUnit(&tlc, 3, 2, ADC1_CHANNEL_6, 10000.0, adc_1_characterisitics, 3, 29);//ADC1_GPIO34_CHANNEL
MotorUnit motor3 = MotorUnit(&tlc, 5, 4, ADC1_CHANNEL_0, 10000.0, adc_1_characterisitics, 22, -29);//ADC1_GPIO36_CHANNEL
MotorUnit motor4 = MotorUnit(&tlc, 7, 6, ADC1_CHANNEL_4, 10000.0, adc_1_characterisitics, 25, 29);//ADC1_GPIO32_CHANNEL
MotorUnit motor5 = MotorUnit(&tlc, 9, 8, ADC1_CHANNEL_7, 10000.0, adc_1_characterisitics, 13, -29);//ADC1_GPIO35_CHANNEL

LimitSwitch flexionLimit = LimitSwitch(GPIO_NUM_14, true);
LimitSwitch extensionLimit = LimitSwitch(GPIO_NUM_16, true);

Ticker motorTimer = Ticker();
Ticker angleTimer = Ticker();
float sampleRateS = 1/60;

void setup(){
    Serial.begin(115200);

    tlc.begin();
    tlc.write();
    // Shuffle our array of speeds for randomization
    printMessage("Speeds before shuffling");
    for (int i = 0; i < numCycles; i++){
        printMessageInt("", cycleSpeeds[i]);
    }
    shuffle (cycleSpeeds.begin(), cycleSpeeds.end(), std::default_random_engine(seed));
    printMessage("Speeds after shuffling");
    for (int i = 0; i < numCycles; i++){
        printMessageInt("",cycleSpeeds[i]);
    }
    #ifndef BOARD_BRINGUP
        calibrateArmMovement();
        motorTimer.attach_ms(100, onControlTimer); //Gets error when faster than ~100ms cycle
        angleTimer.attach(sampleRateS, onAngleTimer);
    #endif
    printMessage("Setup complete");

}

void calibrateArmMovement(){
    while (1){
        delay(1);
        if (flexionLimit.getState()== 0 && extensionLimit.getState()==0){
            motor1.stop();
            printMessage("EStop");
        }
        if (hitFlexionLimit == false){
            if (needSpeed == true){
                motor1.motor->runAtPID(35000);
                printMessage("Set motor speed forward");
                needSpeed = false;
            }
            if (flexionLimit.getState() == 0){
                hitFlexionLimit = true;
                needSpeed = true;
                motor1.setMaxAngle();
                printMessageFloat("Min Angle Set to", motor1.getMaxAngle());
            }
        } else {
            if (needSpeed == true){
                motor1.motor->runAtPID(-35000);
                printMessage("Set motor speed backward");
                needSpeed = false;
            }
            if (extensionLimit.getState() == 0){
                calibrationFinished = true;
                motor1.setMinAngle();
                printMessageFloat("Min Angle Set to", motor1.getMinAngle());
                motor1.stop();
                return;
            }
        }
    }
}

void onControlTimer(){
    motor1.computeSpeed();
    // motor2.computeSpeed();
    // motor3.computeSpeed();
    // motor4.computeSpeed();
    // motor5.computeSpeed();
}

void onAngleTimer(){
    angleLog(motor1.getCurrentAngle());
}

void printCurrentSpeed(){
    switch (motor1.getSpeed()){
        case SLOW:
        case MEDIUMSLOW:
        case MEDIUMFAST:
        case FAST:
            break;
    }
}

void loop(){
#ifndef BOARD_BRINGUP
  delay(5);
  if (flexionLimit.getState()==0 && extensionLimit.getState()==0){
    if (cState != ABORTED){
        motor1.stop();
        printMessage("Aborted Run");
        angleTimer.detach();
        cState = ABORTED;
    }
  }
  switch (cState) {
    case INIT:
        printMessage("Waiting before starting cycling");
        delay (5000);
        printMessage("Beginnning cycling");
        cState = NEXT;
        break;
    case SETTLING:
        break;
    case EXTENDING:
        if (extensionLimit.getState() == 0){
            motor1.stop();
            printMessage("Hit Flexion Limit, Loading Next Movement");
            cState = NEXT;
        }
        break;
    case FLEXING:
        if (flexionLimit.getState() == 0){
            motor1.setExtension();
            printMessage("Hit Flexion Limit, Starting Extension");
            cState = EXTENDING;
        }
        break;
    case FINISHED:
        break;
    case ABORTED:
        break;
    case NEXT:
        cycleNumber += 1;
        if (cycleNumber > (numCycles - 1)){
            motor1.stop();
            printMessage("Finished all movement cycles");
            angleTimer.detach();
            cState = FINISHED;
            break;
        }
        printMessage("Starting next cycle in 3s");
        delay(1000);
        printMessage("Starting next cycle in 2s");
        delay(1000);
        printMessage("Starting next cycle in 1s");
        delay(1000);
        motor1.setSpeed(cycleSpeeds[cycleNumber]);
        motor1.setFlexion();
        motor1.reset();
        cState = FLEXING;
        break;
  }

#else
  printMessage("Pins high:");
  motor1.motor->highZ();
  motor2.motor->highZ();
  motor3.motor->highZ();
  motor4.motor->highZ();
  motor5.motor->highZ();
  motor1.angleSensor->printState();
  delay(5000);
  printMessage("Pins low:");
  motor1.motor->stop();
  motor2.motor->stop();
  motor3.motor->stop();
  motor4.motor->stop();
  motor5.motor->stop();
  delay(5000);
#endif
}
