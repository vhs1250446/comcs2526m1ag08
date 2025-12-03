/*
 * MQTT Client for sending alerts to Command Centre
 * Uses Eclipse Paho MQTT C library
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "mqtt_client.h"

// If Paho MQTT is available, use it. Otherwise, provide stub implementation.
#ifdef USE_PAHO_MQTT
#include <MQTTClient.h>

#define MQTT_BROKER "ssl://2f0de4ef7266438a8f0a43350b1fa391.s1.eu.hivemq.cloud:8883"
#define MQTT_CLIENT_ID "AlertServer-G08"
#define MQTT_USERNAME "comcs2526g08"
#define MQTT_PASSWORD "Bolofofo123"
#define QOS 1
#define TIMEOUT 10000L

static MQTTClient client;
static int mqtt_connected = 0;

int mqtt_init(void) {
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    int rc;
    
    printf("Creating MQTT client: %s (ID: %s)\n", MQTT_BROKER, MQTT_CLIENT_ID);
    
    rc = MQTTClient_create(&client, MQTT_BROKER, MQTT_CLIENT_ID,
                           MQTTCLIENT_PERSISTENCE_DEFAULT, NULL);
    if (rc != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "MQTTClient_create failed, rc=%d\n", rc);
        return -1;
    }
    
    // Configure TLS/SSL (insecure - no cert verification)
    ssl_opts.enableServerCertAuth = 0;
    ssl_opts.verify = 0;
    ssl_opts.enabledCipherSuites = NULL;
    
    conn_opts.keepAliveInterval = 30;
    conn_opts.cleansession = 1;
    conn_opts.username = MQTT_USERNAME;
    conn_opts.password = MQTT_PASSWORD;
    conn_opts.ssl = &ssl_opts;
    conn_opts.connectTimeout = 30;
    conn_opts.automaticReconnect = 1;    // enable auto-reconnect
    conn_opts.minRetryInterval = 1;      // seconds
    conn_opts.maxRetryInterval = 30; 
    conn_opts.MQTTVersion = MQTTVERSION_3_1_1;  // Force MQTT 3.1.1
    
    printf("Attempting MQTT connection (user: %s)...\n", MQTT_USERNAME);
    rc = MQTTClient_connect(client, &conn_opts);
    if (rc != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "MQTTClient_connect failed, rc=%d\n", rc);
        fprintf(stderr, "Error -14: Bad protocol version or protocol not supported\n");
        fprintf(stderr, "Common codes: 1=refused protocol, 2=refused identifier, 3=server unavailable, 5=bad credentials, -14=bad protocol\n");
        MQTTClient_destroy(&client);
        return -1;
    }
    
    mqtt_connected = 1;
    printf("MQTT client connected successfully\n");
    return 0;
}

int mqtt_publish(const char* topic, const char* payload) {
    if (!mqtt_connected) return -1;
    
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;
    
    pubmsg.payload = (void*)payload;
    pubmsg.payloadlen = strlen(payload);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    
    if ((rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token)) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Failed to publish message, return code %d\n", rc);
        return -1;
    }
    
    rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
    printf("MQTT alert published to %s\n", topic);
    return 0;
}

void mqtt_cleanup(void) {
    if (mqtt_connected) {
        MQTTClient_disconnect(client, TIMEOUT);
        MQTTClient_destroy(&client);
        mqtt_connected = 0;
    }
}

#else
// Stub implementation when MQTT library is not available
static int mqtt_enabled = 0;

int mqtt_init(void) {
    printf("MQTT library not available. Alerts will be logged only.\n");
    mqtt_enabled = 0;
    return -1;
}

int mqtt_publish(const char* topic, const char* payload) {
    if (!mqtt_enabled) {
        printf("[MQTT STUB] Would publish to %s:\n%s\n", topic, payload);
    }
    return 0;
}

void mqtt_cleanup(void) {
    // Nothing to clean up in stub
}
#endif
