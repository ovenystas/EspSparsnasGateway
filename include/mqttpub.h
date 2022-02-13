#ifndef MQTTPUB_H
#define MQTTPUB_H

// Publish message as is on topic to MQTT server.
void mqtt_publish(String topic, String message, boolean retained = false);

// Create one JSON message with subject and publish it on topic to MQTT server.
void mqtt_publish(String topic, String subject, String mess, boolean retained = false);

#endif // MQTTPUB_H
