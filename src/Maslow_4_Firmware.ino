#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "html.h"
#include "TLC59711.h"
#include "MotorUnit.h"
#include "LimitSwitch.h"
#include "Ticker.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

// #define BOARD_BRINGUP

unsigned long ourTime = millis();
bool hitFlexionLimit = false;
bool calibrationFinished = false;

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

void setup(){
  Serial.begin(115200);

  tlc.begin();
  tlc.write();
  #ifndef BOARD_BRINGUP
    calibrateArmMovement();
    motorTimer.attach_ms(100, onTimer); //Gets error when faster than ~100ms cycle
  #endif
  Serial.println("Setup complete");

}

void calibrateArmMovement(){
    while (1){
        delay(1);
        if (flexionLimit.getState()== 0 && extensionLimit.getState()==0){
            motor1.stop();
            Serial.println("EStop");
        }
        if (hitFlexionLimit == false){
            motor1.motor->runAtPID(10000);
            if (flexionLimit.getState() == 0){
                hitFlexionLimit = true;
                motor1.setMaxAngle();
                Serial.printf("Max Angle Set to: %f\n", motor1.getMaxAngle());
            }
        } else {
            motor1.motor->runAtPID(-10000);
            if (extensionLimit.getState() == 0){
                calibrationFinished = true;
                motor1.setMinAngle();
                Serial.printf("Min Angle Set to: %f\n", motor1.getMinAngle());
                return;
            }
        }
    }
}

void onTimer(){
  motor1.computeSpeed();
  // motor2.computeSpeed();
  // motor3.computeSpeed();
  // motor4.computeSpeed();
  // motor5.computeSpeed();
}

void loop(){
#ifndef BOARD_BRINGUP
  delay(1);
  if (flexionLimit.getState()==0 && extensionLimit.getState()==0){
    motor1.stop();
    Serial.println("EStop");
  }
  if (flexionLimit.getState() == 0){
    motor1.setExtension();
    Serial.println("Hit Flexion Limit");
  }
  
  if (extensionLimit.getState() == 0){
    motor1.setFlexion();
    Serial.println("Hit Extension Limit");
  }

#else
  Serial.println("Pins high:");
  motor1.motor->highZ();
  motor2.motor->highZ();
  motor3.motor->highZ();
  motor4.motor->highZ();
  motor5.motor->highZ();
  motor1.angleSensor->printState();
  delay(5000);
  Serial.println("Pins low:");
  motor1.motor->stop();
  motor2.motor->stop();
  motor3.motor->stop();
  motor4.motor->stop();
  motor5.motor->stop();
  delay(5000);
#endif
}
