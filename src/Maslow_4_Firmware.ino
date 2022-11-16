#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "html.h"
#include "TLC59711.h"
#include "MotorUnit.h"
#include "LimitSwitch.h"
#include "comands.h"
#include "Ticker.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define BOARD_BRINGUP

const char* ssid = "Leek Soup";
const char* password = "Cranberry Pie";

AsyncWebServer server(80);

unsigned long ourTime = millis();

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

LimitSwitch flexionLimit(22, true);
LimitSwitch extensionLimit = LimitSwitch(25, true);

Ticker motorTimer = Ticker();

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", HTML);
  });

  server.on("/settarget", HTTP_POST, [](AsyncWebServerRequest *request){

    int paramsNr = request->params();
    Serial.println(paramsNr);

    for(int i=0;i<paramsNr;i++){

        AsyncWebParameter* p = request->getParam(i);
        parseCommand(p->name(), p->value());
    }

    request->redirect("/");
  });

  server.on("/position1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor1.getInput(), 5).c_str());
  });
  server.on("/position2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor2.getInput(), 5).c_str());
  });
  server.on("/position3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor3.getInput(), 5).c_str());
  });
  server.on("/position4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor4.getInput(), 5).c_str());
  });
  server.on("/position5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor5.getInput(), 5).c_str());
  });

  server.on("/target1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor1.getSetpoint(), 5).c_str());
  });
  server.on("/target2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor2.getSetpoint(), 5).c_str());
  });
  server.on("/target3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor3.getSetpoint(), 5).c_str());
  });
  server.on("/target4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor4.getSetpoint(), 5).c_str());
  });
  server.on("/target5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor5.getSetpoint(), 5).c_str());
  });

  server.on("/errorDist1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor1.getError(), 5).c_str());
  });
  server.on("/errorDist2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor2.getError(), 5).c_str());
  });
  server.on("/errorDist3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor3.getError(), 5).c_str());
  });
  server.on("/errorDist4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor4.getError(), 5).c_str());
  });
  server.on("/errorDist5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(motor5.getError(), 5).c_str());
  });

  server.on("/proportional", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(proportional, 5).c_str());
  });
  server.on("/integral", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(integral, 5).c_str());
  });
  server.on("/derivative", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(derivative, 5).c_str());
  });

  server.begin();

  tlc.begin();
  tlc.write();
#ifndef BOARD_BRINGUP
  motorTimer.attach_ms(100, onTimer); //Gets error when faster than ~100ms cycle
#endif
  Serial.println("Setup complete");

}

void onTimer(){
  motor1.computePID();
  motor2.computePID();
  motor3.computePID();
  motor4.computePID();
  motor5.computePID();
}

void loop(){
#ifndef BOARD_BRINGUP
  delay(1);
  if(setpointFlag){
    motor1.setSetpoint(setPoint1);
    motor2.setSetpoint(setPoint2);
    motor3.setSetpoint(setPoint3);
    motor4.setSetpoint(setPoint4);
    motor5.setSetpoint(setPoint5);
    setpointFlag = false;
  }
  if(pidFlag){
    motor1.setPIDTune(proportional, integral, derivative);
    motor2.setPIDTune(proportional, integral, derivative);
    motor3.setPIDTune(proportional, integral, derivative);
    motor4.setPIDTune(proportional, integral, derivative);
    motor5.setPIDTune(proportional, integral, derivative);
    pidFlag = false;
  }
  if(modeFlag){
    motor1.setControlMode(updatedMode);
    motor2.setControlMode(updatedMode);
    motor3.setControlMode(updatedMode);
    motor4.setControlMode(updatedMode);
    motor5.setControlMode(updatedMode);
    proportional = motor1.getP();
    integral = motor1.getI();
    derivative = motor1.getD();
    modeFlag = false;
  }
#else
  Serial.println("Pins high:");
  motor1.motor->highZ();
  motor2.motor->highZ();
  motor3.motor->highZ();
  motor4.motor->highZ();
  motor5.motor->highZ();
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
