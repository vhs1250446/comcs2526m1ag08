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
#include <LittleFS.h>
#include <time.h>


// ============================================================================
// CONFIGURATION
// ============================================================================

// Rolling window size
#define WINDOW_SIZE 15

// DHT Sensor Configuration
#define DHTPIN 16
#define DHTTYPE DHT11
#define SENSOR_READ_INTERVAL_MS 1000
float tempBuffer[WINDOW_SIZE];
float humidBuffer[WINDOW_SIZE];

// Storage file path
#define STORAGE_FILE "/storage.dat"

// WiFi Configuration
static const char* ssid = "labs";
static const char* password = "782edcwq#";

// NTP Configuration
#define TIMEZONE "UTC0"
static const char* ntpServer1 = "pool.ntp.org";
static const char* ntpServer2 = "time.nist.gov";
static const long  gmtOffset_sec = 0;
static const int   daylightOffset_sec = 0;

// MQTT Configuration
static const char* mqtt_server = "2f0de4ef7266438a8f0a43350b1fa391.s1.eu.hivemq.cloud";
static const int   mqtt_port = 8883;
static const char* mqtt_username = "comcs2526g08";
static const char* mqtt_password = "Bolofofo123";
static const char* mqtt_topic_sensors = "/comcs/g08/sensors/rp2040";

// UDP Configuration
static const char* udpAddress = "192.168.25.64";
static const int   udpPort = 9999;
static const int   UDP_LOCAL_PORT = 12001;
static const int   UDP_ACK_TIMEOUT_MS = 500;
static const int   UDP_RETRIES = 3;

// UDP QoS Policy (change before flashing)
#define UDP_QOS_BEST_EFFORT 0
#define UDP_QOS_GUARANTEED 1
static const uint8_t UDP_QOS_LEVEL = UDP_QOS_BEST_EFFORT;

// Config delay
#define SETUP_DELAY_MS 5000


// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Storage file header (contains buffer indexes)
typedef struct
{
  int write_index;
  int mqtt_read_index;
  int udp_read_index;
} StorageHeader;

// Telemetry data record
typedef struct
{
  struct tm timestamp;
  float temperature;
  float humidity;
  float avgTemperature;
  float avgHumidity;
} TelemetryData;

struct UdpQoSHeader
{
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
bool bufferInitialized = false;

// Network clients
static WiFiClientSecure rp2040Client;
static PubSubClient client(rp2040Client);
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
  Serial.print("FATAL: ");
  Serial.println(reason);
  Serial.println("Rebooting in 1s...");
  Serial.flush();
  delay(1000);
  rp2040.restart();
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
  JsonDocument doc;
  String payload;

  doc["id"] = "urn:ngsi-ld:WeatherObserved:COMCS-G08-RP2040";
  doc["type"] = "WeatherObserved";
  doc["dateObserved"] = isoTime(data.timestamp);
  doc["temperature"] = data.temperature;
  doc["humidity"] = data.humidity;
  doc["averageTemperature"] = data.avgTemperature;
  doc["averageHumidity"] = data.avgHumidity;

  serializeJson(doc, payload);

  return payload;
}

void getLocalTimeRP(struct tm* timeinfo)
{
  time_t now = time(nullptr);
  localtime_r(&now, timeinfo);
}

// ============================================================================
// STORAGE FUNCTIONS
// ============================================================================

int filesystemInit()
{
  int ret;

  ret = LittleFS.begin();
  if (ret)
    return 0;

  Serial.println("LittleFS begin failed. Formatting...");
  LittleFS.format();

  return LittleFS.begin() ? 0 : -1;
}

int storageInit()
{
  size_t expectedSize = sizeof(StorageHeader) + (size_t)WINDOW_SIZE * sizeof(TelemetryData);
  bool needsCreation = true;

  // Try to load existing storage file
  if (LittleFS.exists(STORAGE_FILE)) {
    File storageFile = LittleFS.open(STORAGE_FILE, "r");
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
    Serial.println("Storage file invalid. Recreating...");
    LittleFS.remove(STORAGE_FILE);
  }

  // Create new storage file
  Serial.println("Creating storage file...");
  File storageFile = LittleFS.open(STORAGE_FILE, "w");
  if (!storageFile) {
    Serial.println("Failed to create storage file!");
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
  Serial.println("Storage file created.");
  return 0;
}

void storageUpdateHeader()
{
  File storage = LittleFS.open(STORAGE_FILE, "r+");
  if (storage) {
    StorageHeader header;
    header.write_index = write_index;
    header.mqtt_read_index = mqtt_read_index;
    header.udp_read_index = udp_read_index;

    storage.seek(0);
    storage.write((uint8_t*)&header, sizeof(StorageHeader));
    storage.close();
  } else {
    Serial.println("Failed to save indexes!");
  }
}

void storagePut(TelemetryData data)
{
  File storage;
  StorageHeader header;
  int mqtt_overwritten = false;
  int udp_overwritten = false;

  storage = LittleFS.open(STORAGE_FILE, "r+");
  if (!storage) {
    Serial.println("[STORAGE] Failed to open storage file for writing!");
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

  Serial.printf("[STORAGE] Saved new entry to slot %d\n", write_index);

  if (mqtt_overwritten) {
    Serial.println("[STORAGE] MQTT oldest entry overwritten.");
  }

  if (udp_overwritten) {
    Serial.println("[STORAGE] UDP oldest entry overwritten.");
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
  Serial.println("[MQTT] DOWN, CONNECTING...");

  String clientId = "RP2040-G08-" + String(random(0xffff), HEX);
  if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("[MQTT] UP.");
    client.subscribe(mqtt_topic_sensors);
    return true;
  }

  Serial.print("[MQTT] CONNECTION FAILED, rc=");
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

  // remember that indexes are part of the shared context
  if (write_index == mqtt_read_index) {
    return; // No data to send
  }

  current_read_index = mqtt_read_index;
  storage = LittleFS.open(STORAGE_FILE, "r");
  if (!storage) {
    Serial.println("[MQTT] Failed to open storage file for reading!");
    return;
  }

  // read data
  storage.seek(DATA_OFFSET(current_read_index));
  storage.read((uint8_t*)&data, sizeof(TelemetryData));
  storage.close();

  // Send data
  payload = jsonfyPayload(data);

  if (client.publish(mqtt_topic_sensors, payload.c_str())) {
    // here we have to first check that the mqtt index has not changed
    // in the meantime (sensor task could have overwritten it)
    if (mqtt_read_index == current_read_index) {
      mqtt_read_index = (mqtt_read_index + 1) % WINDOW_SIZE;
      storageUpdateHeader();
    }
    Serial.println("MQTT publish succeeded.");
  }
  else {
    Serial.println("MQTT publish failed. Will retry.");
    isOnline = false;
  }
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
    Serial.printf("[UDP] Best-effort sent seq=%lu\n", (unsigned long)udp_seq);
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
      Serial.printf("[UDP] Send failed seq=%lu attempt=%d\n",
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
              Serial.printf("[UDP] Guaranteed delivery seq=%lu (attempt %d)\n",
                            (unsigned long)udp_seq, attempt + 1);
              udp_seq++;
              return true;
            }
          }
        }
      }
      delay(10);
    }

    Serial.printf("[UDP] Timeout seq=%lu attempt=%d\n",
                  (unsigned long)udp_seq, attempt + 1);
  }

  Serial.printf("[UDP] Guaranteed delivery FAILED seq=%lu after %d attempts\n",
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

  if (write_index == udp_read_index) {
    return; // No data to send
  }

  current_read_index = udp_read_index;
  storage = LittleFS.open(STORAGE_FILE, "r");
  if (!storage) {
    Serial.println("[UDP] Failed to open storage file for reading!");
    return;
  }

  storage.seek(DATA_OFFSET(current_read_index));
  storage.read((uint8_t*)&data, sizeof(TelemetryData));
  storage.close();

  payload = jsonfyPayload(data);
  bool success = udpSendWithQoS(payload.c_str());

  // Handle result based on QoS level
  if (success || UDP_QOS_LEVEL == UDP_QOS_BEST_EFFORT) {
    // Advance index for successful sends or best-effort mode (accept loss)
    if (udp_read_index == current_read_index) {
      udp_read_index = (udp_read_index + 1) % WINDOW_SIZE;
      storageUpdateHeader();
    }
  } else {
    // Guaranteed delivery failed - will retry next cycle
    Serial.println("[UDP] Guaranteed delivery failed. Will retry next cycle.");
  }
}

// ============================================================================
// NETWORK FUNCTIONS
// ============================================================================

bool wifiCheck()
{
  return WiFi.status() == WL_CONNECTED ? true : false;
}

int networkInit()
{
  // Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("WiFi connecting (SSID: %s)\n", ssid);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.printf("NTP configured (Servers: %s, %s)\n", ntpServer1, ntpServer2);

  // MQTT
  rp2040Client.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  Serial.printf("MQTT configured (Server: %s:%d)\n", mqtt_server, mqtt_port);

  // UDP
  udp.begin(UDP_LOCAL_PORT);
  Serial.printf("UDP configured (QoS: %s)\n",
                UDP_QOS_LEVEL == UDP_QOS_GUARANTEED ? "GUARANTEED" : "BEST_EFFORT");

  return 0;
}

void networkPublish()
{
  mqttPublish();
  udpPublish();
}

// ============================================================================
// SENSOR FUNCTIONS
// ============================================================================

int sensorInit()
{
  dht.begin();
  return 0;
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

void setup()
{
  Serial.begin(9600);
  delay(1000); // wait for serial monitor

  clearScreen();

  REQUIRE_OK(filesystemInit(), "LittleFS initialization failed");
  REQUIRE_OK(storageInit(), "Storage initialization failed");
  REQUIRE_OK(networkInit(), "Network initialization failed");
  REQUIRE_OK(sensorInit(), "Sensor initialization failed");

  delay(SETUP_DELAY_MS); // give time for users to read initial logs and for wifi to kick in
}

void sensorReadings()
{
  TelemetryData currentReading;
  float temperature = 0;
  float humidity = 0;
  int avgIndex = 0;

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    // Initialize buffer with first valid reading to avoid zero-averaging at startup
    if (!bufferInitialized) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        tempBuffer[i] = temperature;
        humidBuffer[i] = humidity;
      }
      bufferInitialized = true;
      Serial.println("Average buffer initialized with first valid reading.");
    }

    tempBuffer[avgIndex]  = temperature;
    humidBuffer[avgIndex] = humidity;
    avgIndex              = (avgIndex + 1) % WINDOW_SIZE;

    getLocalTimeRP(&currentReading.timestamp);
    currentReading.temperature = temperature;
    currentReading.humidity = humidity;
    // Note: average calculation is local to this task
    //       this way we avoid averaging over data outside the
    //       current sensor loop lifecycle
    currentReading.avgTemperature = average(tempBuffer);
    currentReading.avgHumidity = average(humidBuffer);

    storagePut(currentReading);
  }
  Serial.println("Reading: T="
                 + String(currentReading.temperature)
                 + "|H=" + String(currentReading.humidity)
                 + "|AvgT=" + String(currentReading.avgTemperature)
                 + "|AvgH=" + String(currentReading.avgHumidity)
                 + "|Time=" + isoTime(currentReading.timestamp));

  delay(SENSOR_READ_INTERVAL_MS);
}

// Network Loop (runs on Core 0)
void loop()
{
  sensorReadings();
  networkPublish();
}
