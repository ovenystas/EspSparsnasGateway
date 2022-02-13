#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "mqttpub.h"

extern PubSubClient mClient;

void mqtt_publish(String topic, String message, boolean retained) {
  mClient.beginPublish(topic.c_str(), message.length(), retained);
  for (uint i = 0; i < message.length(); i += 64) {
    mClient.print(message.substring(i, i + 64));
  }
  mClient.endPublish();
}

void mqtt_publish(String topic, String subject, String mess, boolean retained) {
  DynamicJsonDocument status(128 + mess.length());
  String mqttMess;
  status[subject] = mess;
  #ifdef DEBUG
    Serial.print("JsonDocSize(mqttpub_status): ");
    Serial.print(object.memoryUsage());
    Serial.println(" Bytes");
  #endif
  serializeJson(status, mqttMess);
  mqtt_publish(topic, mqttMess);
}
