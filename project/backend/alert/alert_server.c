/*
 * Alert Server (UDP Server)
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include "alert_server.h"
#include "lib/mqtt_client.h"
#include "utils/json_parser.h"
#include "utils/kvstore.h"
#include "lib/cJSON.h"

// Global variables
static int server_socket;
static volatile int running = 1;
static struct kv_store device_store;


int main(void) {
  struct addrinfo req;
  struct addrinfo *list;
  struct sockaddr_storage client_addr;
  socklen_t client_addr_len = sizeof(client_addr);
  char buffer[BUFFER_SIZE];
  char client_ip[LOG_BUFFER_SIZE], client_port[LOG_BUFFER_SIZE];
  int ret;

  printf("=== Alert Server Starting ===\n");
  printf("UDP Port: %s\n", UDP_PORT);
  printf("Temp Range: %.1f°C - %.1f°C\n", TEMP_MIN, TEMP_MAX);
  printf("Humidity Range: %.1f%% - %.1f%%\n", HUMIDITY_MIN, HUMIDITY_MAX);
  printf("Temp Diff Threshold: %.1f°C\n", TEMP_DIFF_THRESHOLD);
  printf("Humidity Diff Threshold: %.1f%%\n", HUMIDITY_DIFF_THRESHOLD);

  // Initialize kv store
  if (kv_store_init(&device_store) != 0) {
    fprintf(stderr, "Failed to initialize device store\n");
    exit(EXIT_FAILURE);
  }

  // Initialize MQTT client
  if (mqtt_init() != 0) {
    fprintf(stderr, "Warning: MQTT initialization failed. Alerts will only be logged.\n");
  }

  // request a IPv6 local address will allow both IPv4 and IPv6 clients to use it
  memset(&req, 0, sizeof(req));
  req.ai_family = AF_INET6;
  req.ai_socktype = SOCK_DGRAM;
  req.ai_flags = AI_PASSIVE; // local address
  ret = getaddrinfo(NULL, UDP_PORT , &req, &list);
  if(ret) {
    printf("Failed to get local address, error: %s\n", gai_strerror(ret));
    exit(EXIT_FAILURE);
  }

  server_socket = socket(list->ai_family, list->ai_socktype, list->ai_protocol);
  if (server_socket < 0) {
    printf("Socket creation failed");
    exit(EXIT_FAILURE);
  }

  ret = bind(server_socket,(struct sockaddr *)list->ai_addr, list->ai_addrlen);
  if(ret != 0) {
    printf("Bind failed");
    close(server_socket);
    freeaddrinfo(list);
    exit(EXIT_FAILURE);
  }

  freeaddrinfo(list);

  printf("Server listening on port %s...\n\n", UDP_PORT);

  // Main server loop - receives data and spawns threads
  while (running) {
    int recv_len;
    memset(buffer, 0, BUFFER_SIZE);

    // bytes received may vary from packet to packet
    // due to temp and humidity readings
    recv_len = recvfrom(server_socket, buffer, BUFFER_SIZE - 1, 0,
                        (struct sockaddr*)&client_addr, &client_addr_len);

    if (recv_len < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror("recvfrom failed");
      continue;
    }

    buffer[recv_len] = '\0'; // make sure to stringify the received data

    if(getnameinfo((struct sockaddr *)&client_addr, client_addr_len,
                   client_ip, LOG_BUFFER_SIZE,
                   client_port, LOG_BUFFER_SIZE,
                   NI_NUMERICHOST|NI_NUMERICSERV)) {
      printf("Failed to get client address\n");
    } else {
      printf("Received %d bytes from %s:%s\n", recv_len, client_ip, client_port);
    }

    // Parse UDP QoS header and extract JSON payload
    UdpQoSHeader* qos_hdr = NULL;
    char* json_payload = buffer;
    int json_len = recv_len;

    qos_hdr = (UdpQoSHeader*)buffer;
    json_payload = buffer + sizeof(UdpQoSHeader);
    json_len = recv_len - sizeof(UdpQoSHeader);
    json_payload[json_len] = '\0';

    printf("QoS Header: seq=%u qos=%u flags=0x%02x%s\n",
           qos_hdr->seq, // sequence number
           qos_hdr->qos, // quality of service level (0=best effort, 1=guaranteed)
           qos_hdr->flags, // retransmission flag
           (qos_hdr->flags & 0x01) ? " [RETRANS]" : "");

    // Allocate request structure for thread
    ClientRequest* request = malloc(sizeof(ClientRequest));
    if (!request) {
      printf("Memory allocation failed for client request\n");
      continue;
    }

    request->client_addr = client_addr;
    request->data_len = json_len;
    memcpy(request->data, json_payload, json_len + 1);

    // Store QoS info for ACK handling
    if (qos_hdr) {
      request->qos_level = qos_hdr->qos;
      request->seq_num = qos_hdr->seq;
      request->has_qos = 1;
    } else {
      request->has_qos = 0;
    }

    // Create thread to handle request
    pthread_t thread;
    if (pthread_create(&thread, NULL, handle_client, request) != 0) {
      printf("Thread creation failed\n");
      free(request);
      continue;
    }

    pthread_detach(thread);
  }

  // Cleanup
  close(server_socket);
  kv_store_destroy(&device_store);
  //mqtt_cleanup();

  printf("Server shutting down.\n");
  return 0;
}

void* handle_client(void* arg) {
  ClientRequest* request = (ClientRequest*)arg;
  SensorData data;
  char alert_msg[512];
  int range_violation;
  int fluctuation_alert;

  // Note: retransmission flag alone is not sufficient for deduplication.
  // We will deduplicate by comparing against the last stored reading.

  // Parse JSON data (smartdata model format)
  if (parse_sensor_data(request->data, &data) != 0) {
    fprintf(stderr, "Failed to parse sensor data\n");
    free(request);
    return NULL;
  }
  printf("Parsed: ID        = %sTime      = %.2f°C\n\t\n\tTemp      = %.2f°C\n\tHumidity  = %.2f%%\n\tAvgTemp   = %.2f°C\n\tAvgHum    = %.2f%%\n",
         data.sensor_id, data.timestamp, data.temperature, data.humidity,
         data.avgTemperature, data.avgHumidity);

  // if we already have the reading stored, skip processing and if needed send ACK
  SensorData existing;
  int have_existing = kv_store_get(&device_store, data.sensor_id, &existing) == 0;
  if (have_existing) {
    int same_timestamp = strcmp(existing.timestamp, data.timestamp) == 0;
    int same_values = (existing.temperature == data.temperature) &&
                      (existing.humidity == data.humidity) &&
                      (existing.avgTemperature == data.avgTemperature) &&
                      (existing.avgHumidity == data.avgHumidity);
    if (same_timestamp && same_values) {
      printf("Duplicate reading for %s detected, skipping processing\n", data.sensor_id);
      if (request->has_qos && request->qos_level == UDP_QOS_GUARANTEED) {
        socklen_t addr_len = (request->client_addr.ss_family == AF_INET)
            ? sizeof(struct sockaddr_in)
            : sizeof(struct sockaddr_in6);
        send_ack(&request->client_addr, addr_len, request->seq_num);
      }
      free(request);
      return NULL;
    }
  }

  // Store/update device reading in KV store
  if (kv_store_put(&device_store, data.sensor_id, &data) != 0) {
    fprintf(stderr, "[STORE] Failed to store data for: %s\n", data.sensor_id);
  }

  // Send ACK for guaranteed delivery
  if (request->qos_level == UDP_QOS_GUARANTEED) {
    socklen_t addr_len = (request->client_addr.ss_family == AF_INET)
      ? sizeof(struct sockaddr_in)
      : sizeof(struct sockaddr_in6);
    send_ack(&request->client_addr, addr_len, request->seq_num);
  }

  // Data Range Validation
  range_violation = validate_sensor_data(&data, alert_msg);
  if (range_violation)
    send_range_alert(alert_msg, &data);

  // Data Fluctuation Analysis
  fluctuation_alert = check_fluctuation(&data, alert_msg);
  if (fluctuation_alert)
    send_fluctuation_alert(alert_msg);

  free(request);
  return NULL;
}

int validate_sensor_data(SensorData* data, char* alert_msg) {
  int alert = 0;
  char temp_buf[256];
  alert_msg[0] = '\0';

  // Check temperature range
  if (data->temperature < TEMP_MIN || data->temperature > TEMP_MAX) {
    snprintf(temp_buf, sizeof(temp_buf),
             "Temp OUT OF RANGE: %.2f°C (valid: %.1f-%.1f°C)\n",
             data->temperature, TEMP_MIN, TEMP_MAX);
    strcat(alert_msg, temp_buf);
    alert = 1;
  }

  // Check humidity range
  if (data->humidity < HUMIDITY_MIN || data->humidity > HUMIDITY_MAX) {
    snprintf(temp_buf, sizeof(temp_buf),
             "Hum OUT OF RANGE: %.2f%% (valid: %.1f-%.1f%%)\n",
             data->humidity, HUMIDITY_MIN, HUMIDITY_MAX);
    strcat(alert_msg, temp_buf);
    alert = 1;
  }

  if (alert) {
    snprintf(temp_buf, sizeof(temp_buf),
             "Sensor: %s, Time: %s",
             data->sensor_id, data->timestamp);
    strcat(alert_msg, temp_buf);
    printf("Range Violation Alert: %s\n", alert_msg);
  }

  return alert;
}

int check_fluctuation(SensorData* current, char* alert_msg) {
  int alert = 0;
  int device_counter = 0;
  int stale_counter = 0;
  float max_temp_diff = 0.0f;
  float max_humidity_diff = 0.0f;
  char temp_buf[1024];

  if (!current || !alert_msg) return 0;
  alert_msg[0] = '\0';

  // Parse current timestamp
  struct tm current_tm = {0};
  time_t current_time = 0;
  if (strptime(current->timestamp, "%Y-%m-%dT%H:%M:%SZ", &current_tm) != NULL) {
    current_time = timegm(&current_tm);
  }

  // Iterate over all stored devices and compare with current (exclude self and stale)
  for (int b = 0; b < KV_HASH_SIZE; ++b) {
    struct kv_node* node = device_store.buckets[b];
    while (node) {
      if (strcmp(node->data.sensor_id, current->sensor_id) != 0) {
        // Parse stored sensor timestamp
        struct tm stored_tm = {0};
        time_t stored_time = 0;
        if (strptime(node->data.timestamp, "%Y-%m-%dT%H:%M:%SZ", &stored_tm) != NULL) {
          stored_time = timegm(&stored_tm);
        }

        // Check if reading is stale
        if (current_time > 0 && stored_time > 0) {
          double age = difftime(current_time, stored_time);
          if (age > MAX_READING_AGE_SEC || age < -MAX_READING_AGE_SEC) {
            stale_counter++;
            node = node->next;
            continue; // Skip stale reading
          }
        }

        device_counter++;
        float tdiff = fabs(current->temperature - node->data.temperature);
        float hdiff = fabs(current->humidity - node->data.humidity);
        if (tdiff > max_temp_diff) max_temp_diff = tdiff;
        if (hdiff > max_humidity_diff) max_humidity_diff = hdiff;
        if (tdiff > TEMP_DIFF_THRESHOLD) {
          snprintf(temp_buf, sizeof(temp_buf),
                   "TempDiff: CurrentID=%s OtherID=%s Cur=%.2f Other=%.2f Diff=%.2f (Thr=%.1f)\n",
                   current->sensor_id,
                   node->data.sensor_id,
                   current->temperature,
                   node->data.temperature,
                   tdiff,
                   TEMP_DIFF_THRESHOLD);
          strcat(alert_msg, temp_buf);
          alert = 1;
        }
        if (hdiff > HUMIDITY_DIFF_THRESHOLD) {
          snprintf(temp_buf, sizeof(temp_buf),
                   "HumDiff: CurrentID=%s OtherID=%s Cur=%.2f Other=%.2f Diff=%.2f (Thr=%.1f)\n",
                   current->sensor_id,
                   node->data.sensor_id,
                   current->humidity,
                   node->data.humidity,
                   hdiff, HUMIDITY_DIFF_THRESHOLD);
          strcat(alert_msg, temp_buf);
          alert = 1;
        }
      }
      node = node->next;
    }
  }

  if (alert) {
    snprintf(temp_buf, sizeof(temp_buf),
             "Summary: Sensor=%s Time=%s ComparedDevices=%d Stale=%d MaxTempDiff=%.2f MaxHumDiff=%.2f\n",
             current->sensor_id,
             current->timestamp,
             device_counter,
             stale_counter,
             max_temp_diff,
             max_humidity_diff);
    strcat(alert_msg, temp_buf);
    printf("Fluctuation Alert: %s\n", alert_msg);
  }
  return alert;
}

void send_ack(struct sockaddr_storage* client_addr, socklen_t addr_len, uint32_t seq) {
  char ack_buf[32];
  snprintf(ack_buf, sizeof(ack_buf), "ACK %u", seq);

  int sent = sendto(server_socket, ack_buf, strlen(ack_buf), 0,
                    (struct sockaddr*)client_addr, addr_len);

  if (sent < 0) {
    perror("sendto ACK failed");
  } else {
    printf("Sent ACK for seq=%u\n", seq);
  }
}

void send_range_alert(const char* message, SensorData* data) {
  if (!message || !data) return;

  cJSON *root = cJSON_CreateObject();
  if (!root) return;

  cJSON_AddStringToObject(root, "alertType", "range_violation");
  cJSON_AddStringToObject(root, "sensorId", data->sensor_id);
  cJSON_AddStringToObject(root, "timestamp", data->timestamp);
  cJSON_AddNumberToObject(root, "temperature", data->temperature);
  cJSON_AddNumberToObject(root, "humidity", data->humidity);
  cJSON_AddStringToObject(root, "message", message);

  char* json_str = cJSON_PrintUnformatted(root);
  if (!json_str) { cJSON_Delete(root); return; }
  if (mqtt_publish(MQTT_ALERT_TOPIC, json_str) != 0) {
    fprintf(stderr, "Failed to publish MQTT range alert\n");
  }
  cJSON_free(json_str);
  cJSON_Delete(root);
}

void send_fluctuation_alert(const char* message) {
  if (!message) return;

  cJSON *root = cJSON_CreateObject();
  if (!root) return;

  cJSON_AddStringToObject(root, "alertType", "fluctuation_violation");

  // Timestamp as now (server time) since multiple sensors are involved
  char tsbuf[64];
  time_t now = time(NULL);
  struct tm t;
  gmtime_r(&now, &t);
  snprintf(tsbuf, sizeof(tsbuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  cJSON_AddStringToObject(root, "timestamp", tsbuf);

  // Build sensors array with temp & humidity
  cJSON *arr = cJSON_CreateArray();
  if (!arr) { cJSON_Delete(root); return; }
  for (int b = 0; b < KV_HASH_SIZE; ++b) {
    struct kv_node* node = device_store.buckets[b];
    while (node) {
      cJSON *item = cJSON_CreateObject();
      cJSON_AddStringToObject(item, "sensorId", node->data.sensor_id);
      cJSON_AddNumberToObject(item, "temperature", node->data.temperature);
      cJSON_AddNumberToObject(item, "humidity", node->data.humidity);
      cJSON_AddItemToArray(arr, item);
      node = node->next;
    }
  }
  cJSON_AddItemToObject(root, "sensors", arr);
  cJSON_AddStringToObject(root, "message", message);

  char* json_str = cJSON_PrintUnformatted(root);
  if (!json_str) { cJSON_Delete(root); return; }
  if (mqtt_publish(MQTT_ALERT_TOPIC, json_str) != 0) {
    fprintf(stderr, "Failed to publish MQTT fluctuation alert\n");
  }
  cJSON_free(json_str);
  cJSON_Delete(root);
}
