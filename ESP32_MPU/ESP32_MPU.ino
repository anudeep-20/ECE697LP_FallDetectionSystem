#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <Adafruit_MQTT_Client.h>
#include "model.h"
#include <BluetoothSerial.h>
#include "driver/adc.h"
#include <esp_bt.h>

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define AIO_USERNAME "YOUR_AIO_USERNAME"
#define AIO_KEY "YOUR_AIO_KEY"
#define server "io.adafruit.com"
#define port 1883
#define ARR_SIZE 350

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
WiFiClient esp;
Adafruit_MQTT_Client mqtt(&esp, server, port, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/fall");

Eloquent::ML::Port::RandomForest classifier;
float x[ARR_SIZE][6];
float y[ARR_SIZE];
int idx = 0;
unsigned int FALL = 0;
unsigned int NONFALL = 0;
float mean[6] = {-4.01625365624533,-2.266312013835018,2.8705094902251034,-0.329167052158875,-1.7510799712482121,5.001817981254403};
float scale[6] = {6.1652943827071995,5.556397139349762,5.8867659737956055,3.288372250484766,6.839159786498488,4.5774586571099105};

void setup(void) {

  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  if (!gyro.begin() || !accel.begin()) {
    Serial.println("Ooops, no FXAS21002C or FXOS8700 detected ... Check your wiring!");
    while (1);
  }

  gyro.setRange(GYRO_RANGE_500DPS);
  gyro.setODR(GYRO_ODR_50HZ);
  accel.setSensorMode(ACCEL_ONLY_MODE);
  accel.setAccelRange(ACCEL_RANGE_4G);
  accel.setOutputDataRate(ODR_50HZ);

  setESP32sleep();
}

void loop(void) {
    sensors_event_t gevent, aevent;
    gyro.getEvent(&gevent);
    accel.getEvent(&aevent);

  if(idx < ARR_SIZE){
    x[idx][0] = ((aevent.acceleration.x) - mean[0])/scale[0];
    x[idx][1] = ((aevent.acceleration.y) - mean[1])/scale[1];
    x[idx][2] = ((aevent.acceleration.z) - mean[2])/scale[2];
    x[idx][3] = ((gevent.gyro.x*(180.0 / PI)) - mean[3])/scale[3];
    x[idx][4] = ((gevent.gyro.y*(180.0 / PI)) - mean[4])/scale[4];
    x[idx][5] = ((gevent.gyro.z*(180.0 / PI)) - mean[5])/scale[5];
    y[idx] = classifier.predict(x[idx]);

    if(y[idx])
      FALL++;
    else
      NONFALL++;

    idx++;
  } else {
    Serial.print(FALL);
    Serial.print(",");
    Serial.println(NONFALL);
    if(FALL > NONFALL){
      Serial.println("FALL");
      wakeESP32sleep();
      feed.publish("YES") ? Serial.println("Upload Successful") : Serial.println("Upload Failed");
      setESP32sleep();
    } else {
      Serial.println("NONFALL");
    }

    idx = 0;
    FALL = 0;
    NONFALL = 0;
  }

  delay(20);
}
 
void setESP32sleep() {
    adc_power_off();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    setCpuFrequencyMhz(80);
}
 
void wakeESP32sleep(){
    setCpuFrequencyMhz(240);
    adc_power_on();
    delay(200);
    WiFi.disconnect(false);
    WiFi.mode(WIFI_STA);
 
    delay(200);
 
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    if(!mqtt.connected()){
      Serial.print("Connecting to MQTT");
      while (mqtt.connect()){
        Serial.print(".");
      }
      Serial.println("");
    }
}