/*
 * MQTT Client header
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

int mqtt_init(void);
int mqtt_publish(const char* topic, const char* payload);
void mqtt_cleanup(void);

#endif // MQTT_CLIENT_H
