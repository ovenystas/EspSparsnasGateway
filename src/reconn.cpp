#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "mqttpub.h"

extern PubSubClient mClient;
static const String sensor_id = String(SENSOR_ID);
static const String availability_topic = APPNAME "/" + sensor_id + "/availability";
static const String state_topic = APPNAME "/" + sensor_id + "/state";
static const String discovery_sw_version = __TIME__ " " __DATE__;
static const String discovery_name = APPNAME " " + sensor_id;
static const String discovery_id = APPNAME + sensor_id;
static const String discovery_topic = DISCOVERY_PREFIX "/sensor/" APPNAME "_" + sensor_id;

enum class EntityCategory { NONE, CONFIG, DIAGNOSTIC, SYSTEM };
const char* entity_category_name[] = {
  "None", "config", "diagnostic", "system"
};

enum class StateClass { NONE, MEASUREMENT, TOTAL, TOTAL_INCREASING };
const char* state_class_name[] = {
  "None", "measurement", "total", "total_increasing"
};

static void send_discovery_message(
    const char* device_class,
    const char* measurement,
    const char* unit,
    const char* value_template,
    EntityCategory entity_category,
    StateClass state_class) {

  DynamicJsonDocument device(256);
  device["sw_version"] = discovery_sw_version;
  device["name"] = discovery_name;
  device["identifiers"] = discovery_id;
  device["model"] = APPNAME;
  device["manufacturer"] = F("IKEA");
  #ifdef DEBUG
    Serial.print("JsonDocSize(device): ");
    Serial.print(device.memoryUsage());
    Serial.println(" Bytes");
  #endif

  DynamicJsonDocument config(512);
  config["device"] = device;
  config["device_class"] = device_class;
  if (state_class != StateClass::NONE) {
    config["state_class"] = state_class_name[static_cast<int>(state_class)];
  }
  if (entity_category != EntityCategory::NONE) {
    config["entity_category"] = entity_category_name[static_cast<int>(entity_category)];
  }
  config["unit_of_measurement"] = unit;
  config["name"] = String("ESP Sparsnäs ") + measurement;
  config["unique_id"] = APPNAME "_" + sensor_id + '_' + measurement;
  config["state_topic"] = state_topic;
  config["json_attributes_topic"] = state_topic;
  config["availability_topic"] = availability_topic;
  config["value_template"] = value_template;
  #ifdef DEBUG
    Serial.print("JsonDocSize(config): ");
    Serial.print(config.memoryUsage());
    Serial.println(" Bytes");
  #endif

  String mqttMess;
  serializeJson(config, mqttMess);
  #ifdef DEBUG
    Serial.print(F("Discovery["));
    Serial.print(measurement);
    Serial.print("]: ");
    Serial.println(mqttMess);
  #endif
  mqtt_publish(String(discovery_topic) + '/' + measurement + "/config", mqttMess, true);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mClient.connected()) {
    #ifdef DEBUG
      Serial.print(F("Attempting MQTT connection..."));
    #endif
    if (mClient.connect(APPNAME, MQTT_USERNAME, MQTT_PASSWORD, availability_topic.c_str(), 0, true, "offline")) {
      #ifdef DEBUG
        Serial.println(F("Connected to Mqtt broker as " APPNAME));
      #endif

      send_discovery_message("power", "W", "W", "{{ value_json.watt | round(0) }}",
        EntityCategory::NONE, StateClass::MEASUREMENT);

      send_discovery_message("energy", "kWh", "kWh", "{{ value_json.total | round(1) }}",
        EntityCategory::NONE, StateClass::TOTAL);

      send_discovery_message("battery", "battery", "%", "{{ value_json.battery }}",
        EntityCategory::DIAGNOSTIC, StateClass::NONE);

      send_discovery_message("signal_strength", "rssi", "dBm", "{{ value_json.rssi }}",
        EntityCategory::DIAGNOSTIC, StateClass::NONE);

      mqtt_publish(availability_topic, "online", true);
    } else {
      Serial.println(F("MQTT connection to " MQTT_SERVER " failed, check your settings."));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
