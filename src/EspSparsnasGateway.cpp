/*
  * https://github.com/bphermansson/EspSparsnasGateway
  *
  * Based on code from user Sommarlov @ EF: http://elektronikforumet.com/forum/viewtopic.php?f=2&t=85006&start=255#p1357610
  * Which in turn is based on Strigeus work: https://github.com/strigeus/sparsnas_decoder
  *
 */
#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
/*------------------------------*/
#include "RFM69registers.h"
#include "RFM69functions.h"
#include "reconn.h"
#include "mqttpub.h"
#include "ntp.h"
#include "settings.h"

WiFiClient wClient;
PubSubClient mClient(wClient);

static const String mqtt_status_topic = MQTT_STATUS_TOPIC;
static const String mqtt_debug_topic = MQTT_DEBUG_TOPIC;

// Make it possible to read Vcc from code
ADC_MODE(ADC_VCC);

static Ticker blinkerRed;
static Ticker blinkerGreen;

static void changeStateLED_RED();
static void changeStateLED_GREEN();

/* ----------------------------------------------------*/

void setup() {
  String mqttMess;

  Serial.begin(SERIALSPEED);
  Serial.println("Welcome to " APPNAME);

  pinMode (LED_RED, OUTPUT);
  pinMode (LED_GREEN, OUTPUT);
  pinMode (LED_BLUE, OUTPUT);

  // Test leds
  analogWrite(LED_RED, LED_RED_BRIGHTNESS);
  delay(500);
  analogWrite(LED_GREEN, LED_GREEN_BRIGHTNESS);
  delay(500);
  analogWrite(LED_BLUE, LED_BLUE_BRIGHTNESS);
  delay(1500);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
  delay(500);

  blinkerGreen.attach(0.5, changeStateLED_GREEN);

  #ifdef SHOW_BAD_PACKETS
     Serial.println(F("SHOW_BAD_PACKETS on"));
  #else
     Serial.println(F("SHOW_BAD_PACKETS off"));
  #endif

  #ifdef DEBUG
     Serial.println(F("Debug on"));
     Serial.print (F("Vcc="));
     Serial.print(ESP.getVcc());
     Serial.println("mV");
     Serial.println (F("Set up WiFi..."));
  #else
     Serial.println(F("Debug off"));   
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFISSID, WIFIPASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(F("Connecting to " WIFISSID "..."));
    Serial.println(F("Connection failed, check your settings!"));
    Serial.println(F("Rebooting..."));

    blinkerRed.attach(0.5, changeStateLED_RED);
    delay(5000);
    ESP.restart();
  }

  WiFi.hostname(APPNAME);

  // Setup Mqtt connection
  mClient.setServer(MQTT_SERVER, MQTT_PORT);
  reconnect();

  mqttMess = "Welcome to EspSparsnasGateway, compiled at " __DATE__ " " __TIME__ ".\n"
    "Mqtt topics: " + mqtt_status_topic + ", " + mqtt_debug_topic + "\n"
    + "IP: " + WiFi.localIP().toString();
  #ifdef DEBUG
    Serial.println(mqttMess);
    Serial.println(mqtt_status_topic);
  #endif
  mqtt_publish(mqtt_debug_topic, F("Device"), mqttMess);

  // Hostname defaults to esp8266-[ChipID], change this
  ArduinoOTA.setHostname(APPNAME);

  ArduinoOTA.onStart([]() {
    Serial.println(F("Start"));
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nEnd"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", progress / (total / 100));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.print(F("Auth"));
    else if (error == OTA_BEGIN_ERROR) Serial.print(F("Begin"));
    else if (error == OTA_CONNECT_ERROR) Serial.print(F("Connect"));
    else if (error == OTA_RECEIVE_ERROR) Serial.print(F("Receive"));
    else if (error == OTA_END_ERROR) Serial.print(F("End"));
    Serial.println(F(" Failed"));
  });
  
  ArduinoOTA.begin();

  setup_NTP();

  mqttMess = F("Over The Air programming enabled, host: " APPNAME);
  #ifdef DEBUG
    Serial.println(mqttMess);
  #endif
  mqtt_publish(mqtt_debug_topic, F("Device"), mqttMess);

  mqttMess = "Settings:\n"
    "  Senderid: " + String(SENSOR_ID) + "\n"
    "  Frequency: " + String(FREQUENCY) + " Hz";
  #ifdef DEBUG
    Serial.println(mqttMess);
  #endif
  mqtt_publish(mqtt_debug_topic, F("Device"), mqttMess);

  if (!initialize(FREQUENCY)) {
    mqttMess =  F("Unable to initialize the radio. Exiting.");
    blinkerRed.attach(0.3, changeStateLED_RED);
    #ifdef DEBUG
      Serial.println(mqttMess);
    #endif
    mqtt_publish(mqtt_debug_topic, F("Radio"), mqttMess);
    while (1) {
      yield();
    }
  }
  else {
    mqttMess = "Radio initialized.\n"
      "Listening on " + String(getFrequency()) + " Hz. Done in setup.";
    #ifdef DEBUG
      Serial.println(mqttMess);
    #endif
    mqtt_publish(mqtt_debug_topic, F("Radio"), mqttMess);
  }

// All ok
  blinkerGreen.detach();
  analogWrite(LED_GREEN, LED_GREEN_BRIGHTNESS);
}

void loop() {
  ArduinoOTA.handle();
  if (!mClient.connected()) {
    reconnect();
  }
  mClient.loop();
  //delay(10);
  // Note! This routine is necessary, don't remove it!
  receiveDone();
}

void changeStateLED_RED()
{
  digitalWrite(LED_RED, !(digitalRead(LED_RED)));  //Invert Current State of LED
}

void changeStateLED_GREEN()
{
  digitalWrite(LED_GREEN, !(digitalRead(LED_GREEN)));  //Invert Current State of LED
}
