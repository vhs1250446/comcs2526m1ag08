/*
  ESP32 Environmental Monitoring System
  All-in-one implementation for COMCS project
*/

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <FS.h>
#include <SPIFFS.h>
#include <time.h>


// ============================================================================
// CONFIGURATION
// ============================================================================

// Rolling window size
#define WINDOW_SIZE 15

// DHT Sensor Configuration
#define DHTPIN 21
#define DHTTYPE DHT11
#define SENSOR_READ_INTERVAL_MS 1000

// Storage file path
#define STORAGE_FILE "/storage.dat"

// WiFi Configuration
//static const char* ssid = "labs";
//static const char* password = "782edcwq#";
static const char* ssid = "jolteon";
static const char* password = "scone3-rekindle-lisp";

// NTP Configuration
static const char* ntpServer1 = "pool.ntp.org";
static const char* ntpServer2 = "time.nist.gov";
static const long  gmtOffset_sec = 0;
static const int   daylightOffset_sec = 0;

// MQTT Configuration
static const char* mqtt_server = "2f0de4ef7266438a8f0a43350b1fa391.s1.eu.hivemq.cloud";
static const int   mqtt_port = 8883;
static const char* mqtt_username = "comcs2526g08";
static const char* mqtt_password = "Bolofofo123";
static const char* mqtt_topic_sensors = "/comcs/g08/sensors/esp32";
static const char* mqtt_topic_qos = "/comcs/go8/sensors/eps32/qos";

// UDP Configuration
static const char* udpAddress = "192.168.25.64";
static const int   udpPort = 9999;
static const int   UDP_LOCAL_PORT = 12001;
static const int   UDP_ACK_TIMEOUT_MS = 500;
static const int   UDP_RETRIES = 3;

// UDP QoS Policy (change before flashing)
#define UDP_QOS_BEST_EFFORT 0
#define UDP_QOS_GUARANTEED 1
volatile uint8_t UDP_QOS_LEVEL = UDP_QOS_GUARANTEED;

// Setup delay
#define SETUP_DELAY_MS 5000


// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Storage file header (contains buffer indexes)
typedef struct {
  int write_index;
  int mqtt_read_index;
  int udp_read_index;
} StorageHeader;

// Telemetry data record
typedef struct {
  struct tm timestamp;
  float temperature;
  float humidity;
  float avgTemperature;
  float avgHumidity;
} TelemetryData;

struct UdpQoSHeader {
  uint32_t seq;
  uint8_t qos;
  uint8_t flags;  // bit0: retransmission
} __attribute__((packed));


// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Storage indexes
int write_index = 0;
int mqtt_read_index = 0;
int udp_read_index = 0;

// State
bool isOnline = false;
bool timeSynchronized = false;

// Sensor
DHT dht(DHTPIN, DHTTYPE);

// FreeRTOS Handles
SemaphoreHandle_t STORAGE_MUTEX;
TaskHandle_t sensorTaskHandle;

// Network clients
static WiFiClientSecure espClient;
static PubSubClient client(espClient);
static WiFiUDP udp;

// UDP QoS tracking
static uint32_t udp_seq = 0;


// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Data offset helper
#define DATA_OFFSET(slot) (sizeof(StorageHeader) + (slot) * sizeof(TelemetryData))

#define REQUIRE_OK(call, msg) do { if ((call) != 0) fatalReboot(msg); } while(0)

void clearScreen()
{
  for (int i = 0; i < 50; i++) {
    Serial.println();
  }
}

void fatalReboot(const char* reason)
{
  Serial.print(F("FATAL: "));
  Serial.println(reason);
  Serial.println(F("Rebooting in 1s..."));
  Serial.flush();
  delay(1000);
  ESP.restart();
  while (true) { delay(100); }
}

String isoTime(struct tm timestamp)
{
  char data[100];

  sprintf(data, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          timestamp.tm_year + 1900,
          timestamp.tm_mon + 1,
          timestamp.tm_mday,
          timestamp.tm_hour,
          timestamp.tm_min,
          timestamp.tm_sec);

  return String(data);
}

float average(float* buffer)
{
  float sum = 0.0;
  for(int i = 0; i < WINDOW_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / WINDOW_SIZE;
}

String jsonfyPayload(TelemetryData data)
{
  StaticJsonDocument<512> doc;
  String payload;

  doc["id"] = "urn:ngsi-ld:WeatherObserved:COMCS-G08-ESP32";
  doc["type"] = "WeatherObserved";
  doc["dateObserved"] = isoTime(data.timestamp);
  doc["temperature"] = data.temperature;
  doc["humidity"] = data.humidity;
  doc["averageTemperature"] = data.avgTemperature;
  doc["averageHumidity"] = data.avgHumidity;

  serializeJson(doc, payload);

  return payload;
}

// ============================================================================
// STORAGE FUNCTIONS
// ============================================================================

int mutexInit()
{
  STORAGE_MUTEX = xSemaphoreCreateMutex();
  return STORAGE_MUTEX != NULL ? 0 : -1;
}

int filesystemInit(bool formatOnFail)
{
  return SPIFFS.begin(formatOnFail) ? 0 : -1;
}

int storageInit()
{
  size_t expectedSize = sizeof(StorageHeader) + (size_t)WINDOW_SIZE * sizeof(TelemetryData);
  bool needsCreation = true;

  // Try to load existing storage file
  if (SPIFFS.exists(STORAGE_FILE)) {
    File storageFile = SPIFFS.open(STORAGE_FILE, "r");
    if (storageFile && storageFile.size() == expectedSize) {
      StorageHeader header;
      storageFile.read((uint8_t*)&header, sizeof(StorageHeader));
      storageFile.close();

      write_index = header.write_index;
      mqtt_read_index = header.mqtt_read_index;
      udp_read_index = header.udp_read_index;

      Serial.printf("Loaded indexes: Write=%d, MQTT_R=%d, UDP_R=%d\n",
        write_index, mqtt_read_index, udp_read_index);
      return 0;
    }
    if (storageFile) storageFile.close();
    Serial.println(F("Storage file invalid. Recreating..."));
    SPIFFS.remove(STORAGE_FILE);
  }

  // Create new storage file
  Serial.println(F("Creating storage file..."));
  File storageFile = SPIFFS.open(STORAGE_FILE, "w");
  if (!storageFile) {
    Serial.println(F("Failed to create storage file!"));
    return -1;
  }

  StorageHeader header = {0, 0, 0};
  storageFile.write((uint8_t*)&header, sizeof(StorageHeader));

  TelemetryData emptyEntry = {0};
  for (int i = 0; i < WINDOW_SIZE; i++) {
    storageFile.write((uint8_t*)&emptyEntry, sizeof(TelemetryData));
  }
  storageFile.close();

  write_index = mqtt_read_index = udp_read_index = 0;
  Serial.println(F("Storage file created."));
  return 0;
}

void storageUpdateHeader()
{
  File storage = SPIFFS.open(STORAGE_FILE, "r+");
  if (storage) {
    StorageHeader header;
    header.write_index = write_index;
    header.mqtt_read_index = mqtt_read_index;
    header.udp_read_index = udp_read_index;

    storage.seek(0);
    storage.write((uint8_t*)&header, sizeof(StorageHeader));
    storage.close();
  } else {
    Serial.println(F("Failed to save indexes!"));
  }
}

void storagePut(TelemetryData data)
{
  File storage;
  StorageHeader header;
  bool mqtt_overwritten = false;
  bool udp_overwritten = false;

  if (xSemaphoreTake(STORAGE_MUTEX, portMAX_DELAY) != pdTRUE)
    return;

  storage = SPIFFS.open(STORAGE_FILE, "r+");
  if (!storage) {
      Serial.println(F("Core 1: [STORAGE] Failed to open storage file for writing!"));
      xSemaphoreGive(STORAGE_MUTEX);
      return;
    }

    // Append new readings
    storage.seek(DATA_OFFSET(write_index));
    storage.write((uint8_t*)&data, sizeof(TelemetryData));

    // update indexes
    write_index = (write_index + 1) % WINDOW_SIZE;

    // check for overwrites
    // note 1: due to storage circularity
    //         when an overwrite happens
    //         WINDOW_SIZE readings will be sent over the network
    if (write_index == mqtt_read_index) {
      mqtt_read_index = (mqtt_read_index + 1) % WINDOW_SIZE;
      mqtt_overwritten = true;
    }
    if (write_index == udp_read_index) {
      udp_read_index = (udp_read_index + 1) % WINDOW_SIZE;
      udp_overwritten = true;
    }

    // Update header
    header.write_index = write_index;
    header.mqtt_read_index = mqtt_read_index;
    header.udp_read_index = udp_read_index;
    storage.seek(0);
    storage.write((uint8_t*)&header, sizeof(StorageHeader));

    storage.close();

    xSemaphoreGive(STORAGE_MUTEX);

    Serial.printf("Core 1: [STORAGE] Saved new entry to slot %d\n", write_index);

    if (mqtt_overwritten) {
      Serial.println(F("Core 1: [STORAGE] MQTT oldest entry overwritten."));
    }

    if (udp_overwritten) {
      Serial.println(F("Core 1: [STORAGE] UDP oldest entry overwritten."));
    }
}


// ============================================================================
// MQTT FUNCTIONS
// ============================================================================

bool mqttCheckConnectivity()
{
  if (client.connected()) {
    client.loop();
    return true;
  }
  Serial.println(F("Core 0: [MQTT] DOWN, CONNECTING..."));

  String clientId = "ESP32-G08-" + String(random(0xffff), HEX);
  if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
    Serial.println(F("Core 0: [MQTT] UP."));
    client.subscribe(mqtt_topic_qos);
    return true;
  }

  Serial.print(F("Core 0: [MQTT] CONNECTION FAILED, rc="));
  Serial.println(client.state());

  return false;
}

void mqttPublish()
{
  File storage;
  TelemetryData data;
  String payload;
  int current_read_index;

  if (!mqttCheckConnectivity())
    return;

  if (xSemaphoreTake(STORAGE_MUTEX, (TickType_t)10) != pdTRUE)
    return;

  // remember that indexes are part of the shared context
  if (write_index == mqtt_read_index) {
    xSemaphoreGive(STORAGE_MUTEX);
    return; // No data to send
  }

  current_read_index = mqtt_read_index;
  storage = SPIFFS.open(STORAGE_FILE, "r");
  if (!storage) {
    xSemaphoreGive(STORAGE_MUTEX);
    Serial.println(F("Core 0: [MQTT] Failed to open storage file for reading!"));
    return;
  }

  // read data
  storage.seek(DATA_OFFSET(current_read_index));
  storage.read((uint8_t*)&data, sizeof(TelemetryData));
  storage.close();

  xSemaphoreGive(STORAGE_MUTEX);

  // Send data
  payload = jsonfyPayload(data);

  if (client.publish(mqtt_topic_sensors, payload.c_str())) {
    // here we have to first check that the mqtt index has not changed
    // in the meantime (sensor task could have overwritten it)
    if (xSemaphoreTake(STORAGE_MUTEX, portMAX_DELAY) == pdTRUE) {
      if (mqtt_read_index == current_read_index) {
        mqtt_read_index = (mqtt_read_index + 1) % WINDOW_SIZE;
        storageUpdateHeader();
      }
      xSemaphoreGive(STORAGE_MUTEX);
      Serial.println(F("Core 0: MQTT publish succeeded."));
    }
  } else {
    Serial.println(F("Core 0: MQTT publish failed. Will retry."));
    isOnline = false;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char buffer[64];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';
  long qosLevel = strtol(buffer, nullptr, 10);
  if (qosLevel != UDP_QOS_BEST_EFFORT && qosLevel != UDP_QOS_GUARANTEED) {
    Serial.println("Core 0: [MQTT] Invalid QoS level received: " + String(qosLevel));
    return;
  }
  UDP_QOS_LEVEL = (uint8_t)qosLevel;
  Serial.println("Core 0: [MQTT] Received QoS level change request: " + String(qosLevel));
}

// ============================================================================
// UDP FUNCTIONS
// ============================================================================

bool udpSendWithBestEffort(const char* payload)
{
  UdpQoSHeader hdr;
  hdr.seq = udp_seq;
  hdr.qos = UDP_QOS_BEST_EFFORT;
  hdr.flags = 0;

  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t*)&hdr, sizeof(hdr));
  udp.print(payload);
  bool sent = udp.endPacket() != 0;
  if (sent) {
    Serial.printf("Core 0: [UDP] Best-effort sent seq=%lu\n", (unsigned long)udp_seq);
    udp_seq++;
  }
  return sent;
}

bool udpSendWithGuaranteedDelivery(const char* payload)
{
  UdpQoSHeader hdr;
  hdr.seq = udp_seq;
  hdr.qos = UDP_QOS_GUARANTEED;
  hdr.flags = 0;

  // Retry with ACK
  for (int attempt = 0; attempt < UDP_RETRIES; attempt++) {
    hdr.flags = (attempt > 0) ? 0x01 : 0x00;

    udp.beginPacket(udpAddress, udpPort);
    udp.write((uint8_t*)&hdr, sizeof(hdr));
    udp.print(payload);
    if (!udp.endPacket()) {
      Serial.printf("Core 0: [UDP] Send failed seq=%lu attempt=%d\n",
                    (unsigned long)udp_seq, attempt + 1);
      continue;
    }

    // Wait for ACK
    unsigned long start = millis();
    while (millis() - start < UDP_ACK_TIMEOUT_MS) {
      int len = udp.parsePacket();
      if (len > 0) {
        char ackBuf[32];
        int n = udp.read(ackBuf, sizeof(ackBuf) - 1);
        if (n > 0) {
          ackBuf[n] = 0;
          if (strncmp(ackBuf, "ACK ", 4) == 0) {
            uint32_t ackSeq = strtoul(ackBuf + 4, nullptr, 10);
            if (ackSeq == udp_seq) {
              Serial.printf("Core 0: [UDP] Guaranteed delivery seq=%lu (attempt %d)\n",
                           (unsigned long)udp_seq, attempt + 1);
              udp_seq++;
              return true;
            }
          }
        }
      }
      delay(10);
    }

    Serial.printf("Core 0: [UDP] Timeout seq=%lu attempt=%d\n",
                  (unsigned long)udp_seq, attempt + 1);
  }

  Serial.printf("Core 0: [UDP] Guaranteed delivery FAILED seq=%lu after %d attempts\n",
                (unsigned long)udp_seq, UDP_RETRIES);
  return false;
}

bool udpSendWithQoS(const char* payload)
{
  if (UDP_QOS_LEVEL == UDP_QOS_BEST_EFFORT) {
    return udpSendWithBestEffort(payload);
  } else {
    return udpSendWithGuaranteedDelivery(payload);
  }
}

void udpPublish()
{
  File storage;
  TelemetryData data;
  String payload;
  int current_read_index;

  if (xSemaphoreTake(STORAGE_MUTEX, (TickType_t)10) != pdTRUE)
    return;

  if (write_index == udp_read_index) {
    xSemaphoreGive(STORAGE_MUTEX);
    return; // No data to send
  }

  current_read_index = udp_read_index;
  storage = SPIFFS.open(STORAGE_FILE, "r");
  if (!storage) {
    xSemaphoreGive(STORAGE_MUTEX);
    Serial.println(F("Core 0: [UDP] Failed to open storage file for reading!"));
    return;
  }

  storage.seek(DATA_OFFSET(current_read_index));
  storage.read((uint8_t*)&data, sizeof(TelemetryData));
  storage.close();

  xSemaphoreGive(STORAGE_MUTEX);

  payload = jsonfyPayload(data);
  bool success = udpSendWithQoS(payload.c_str());

  // Handle result based on QoS level
  if (success || UDP_QOS_LEVEL == UDP_QOS_BEST_EFFORT) {
    // Advance index for successful sends or best-effort mode (accept loss)
    if (xSemaphoreTake(STORAGE_MUTEX, portMAX_DELAY) == pdTRUE) {
      if (udp_read_index == current_read_index) {
        udp_read_index = (udp_read_index + 1) % WINDOW_SIZE;
        storageUpdateHeader();
      }
      xSemaphoreGive(STORAGE_MUTEX);
    }
  } else {
    // Guaranteed delivery failed - will retry next cycle
    Serial.println(F("Core 0: [UDP] Guaranteed delivery failed. Will retry next cycle."));
  }
}


// ============================================================================
// NETWORK FUNCTIONS
// ============================================================================

bool wifiCheck()
{
  return WiFi.status() == WL_CONNECTED ? true : false;
}

int network_init()
{
  // Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("WiFi connecting (SSID: %s)\n", ssid);

  // NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.printf("NTP configured (Servers: %s, %s)\n", ntpServer1, ntpServer2);

  // MQTT
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  Serial.printf("MQTT configured (Server: %s:%d)\n", mqtt_server, mqtt_port);

  // UDP
  udp.begin(UDP_LOCAL_PORT);
  Serial.printf("UDP configured (QoS: %s)\n",
                UDP_QOS_LEVEL == UDP_QOS_GUARANTEED ? "GUARANTEED" : "BEST_EFFORT");


  return 0;
}

void network_publish()
{
  mqttPublish();
  udpPublish();
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

int sensor_init()
{
  dht.begin();
  return 0;
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(1000); // wait for serial monitor

  clearScreen();

  REQUIRE_OK(filesystemInit(true), "SPIFFS initialization failed");
  REQUIRE_OK(mutexInit(), "Mutex initialization failed");
  REQUIRE_OK(storageInit(), "Storage initialization failed");
  REQUIRE_OK(network_init(), "Network initialization failed");
  REQUIRE_OK(sensor_init(), "Sensor initialization failed");

  delay(SETUP_DELAY_MS); // give time for users to read initial logs and for wifi to kick in

  Serial.println(F("Starting sensor-reading task on Core 1..."));
  xTaskCreatePinnedToCore(sensor_loop, "SensorTask", 4096, NULL, 1, &sensorTaskHandle, 1);

  Serial.println(F("Starting network task on Core 0...")) ;
}

void sensor_loop(void *pvParameters)
{
  TelemetryData currentReading;
  float tempBuffer[WINDOW_SIZE];
  float humidBuffer[WINDOW_SIZE];
  float temperature = 0;
  float humidity = 0;
  int avgIndex = 0;
  bool bufferInitialized = false;

  while (1) {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Core 1: Failed to read from DHT sensor!");
    } else {
      // Initialize buffer with first valid reading to avoid zero-averaging at startup
      if (!bufferInitialized) {
        for (int i = 0; i < WINDOW_SIZE; i++) {
          tempBuffer[i] = temperature;
          humidBuffer[i] = humidity;
        }
        bufferInitialized = true;
        Serial.println("Core 1: Average buffer initialized with first valid reading.");
      }

      tempBuffer[avgIndex]  = temperature;
      humidBuffer[avgIndex] = humidity;
      avgIndex              = (avgIndex + 1) % WINDOW_SIZE;

      getLocalTime(&currentReading.timestamp);
      currentReading.temperature = temperature;
      currentReading.humidity = humidity;
      // Note: average calculation is local to this task
      //       this way we avoid averaging over data outside the
      //       current sensor loop lifecycle
      currentReading.avgTemperature = average(tempBuffer);
      currentReading.avgHumidity = average(humidBuffer);

      storagePut(currentReading);
    }
    Serial.println("Core 1: Reading: T="
                  + String(currentReading.temperature)
                  + "|H=" + String(currentReading.humidity)
                  + "|AvgT=" + String(currentReading.avgTemperature)
                  + "|AvgH=" + String(currentReading.avgHumidity));

    vTaskDelay(SENSOR_READ_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void loop()
{
  bool wifiStatus = false;

  wifiStatus = wifiCheck();

  if (wifiStatus) {
    network_publish();
  }
  else {
    Serial.print(F("Core 0: [WIFI] Connecting..."));
    delay(500);
  }
}
