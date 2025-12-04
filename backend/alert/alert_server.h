/*
 * Alert Server Header
 * Function prototypes and configuration for the alert server
 */

#ifndef ALERT_SERVER_H
#define ALERT_SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>

// Configuration
#define UDP_PORT "9999"
#define BUFFER_SIZE 4096
#define LOG_BUFFER_SIZE 1024
#define MAX_CLIENTS 10
#define MQTT_ALERT_TOPIC "/comcs/g08/alerts"

// Validation thresholds
#define TEMP_MIN 0.0
#define TEMP_MAX 50.0
#define HUMIDITY_MIN 20.0
#define HUMIDITY_MAX 80.0

// Fluctuation thresholds
#define TEMP_DIFF_THRESHOLD 2.0
#define HUMIDITY_DIFF_THRESHOLD 5.0

// Timestamp staleness threshold (seconds)
#define MAX_READING_AGE_SEC 30

// UDP QoS Header (matches ESP32 format)
#define UDP_QOS_BEST_EFFORT 0
#define UDP_QOS_GUARANTEED 1

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint8_t qos;
    uint8_t flags;  // bit0: retransmission
} UdpQoSHeader;

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

// Client request structure
typedef struct {
    struct sockaddr_storage client_addr;
    char data[BUFFER_SIZE];
    int data_len;
    int has_qos;
    uint8_t qos_level;
    uint32_t seq_num;
} ClientRequest;

// Function prototypes
void* handle_client(void* arg);
int validate_sensor_data(SensorData* data, char* alert_msg);
int check_fluctuation(SensorData* data, char* alert_msg);
void send_range_alert(const char* message, SensorData* data);
void send_fluctuation_alert(const char* message);
void send_ack(struct sockaddr_storage* client_addr, socklen_t addr_len, uint32_t seq);
int parse_sensor_data(const char* json, SensorData* data);
int mqtt_init(void);
int mqtt_publish(const char* topic, const char* payload);
void mqtt_cleanup(void);

#endif // ALERT_SERVER_H
