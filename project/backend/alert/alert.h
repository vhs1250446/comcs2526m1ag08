/*
 * Alert Server header file
 * Data structures and function declarations for Epic 2
 */

#ifndef ALERT_H
#define ALERT_H

#include <sys/socket.h>
#include <netinet/in.h>


// Sensor data structure (smartdata model)
typedef struct {
    char sensor_id[128];
    char timestamp[64];
    char type[64];
    float temperature;
    float humidity;
    float avgTemperature;
    float avgHumidity;
} SensorData;

// Function declarations
int parse_sensor_data(const char* json, SensorData* data);
int mqtt_init(void);
int mqtt_publish(const char* topic, const char* payload);
void mqtt_cleanup(void);

#endif // ALERT_H