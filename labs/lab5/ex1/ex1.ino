#include <FS.h>
#include "SPIFFS.h"
#include "Arduino.h" // for identify the core

#define LED_BUILTIN 2

const int LOG_SIZE = 10;
const char* filename = "/logfile.bin";

struct LogEntry {
  uint8_t pos;
  uint32_t timestamp;
  uint32_t freeHeap;
  size_t totalSPIFFS; //It is large enough to represent the maximum possible size of any object that can be allocated on that system
  size_t usedSPIFFS;
  size_t availableSPIFFS;
  uint8_t iCore;

  int ledStatus;
};

void writeLogEntry(int index, const LogEntry entry) {
  File file;

  file = SPIFFS.open(filename, FILE_WRITE); // w+ opens file for reads & writes, if not exists, creates it
  if (!file) {
    Serial.println("Failed to open file for writting");
    return;
  }
  
  file.seek(index * sizeof(entry), SeekSet);
  file.write((const uint8_t*)&entry, sizeof(entry));
  file.close();
}

void readLogEntries(const char* filename) {
  File file;

  file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  while (file.available() >= sizeof(LogEntry)) {
    LogEntry entry;
    file.read((uint8_t*)&entry, sizeof(entry));
    Serial.println("Printing from SPIFFS");
    Serial.printf("file:pos:%u ts:%u heap:%u SPIFFS total:%u used:%u avail:%u core:%u ledStatus:%u\n",
                  entry.pos,
                  entry.timestamp,
                  entry.freeHeap,
                  (uint32_t)entry.totalSPIFFS,
                  (uint32_t)entry.usedSPIFFS,
                  (uint32_t)entry.availableSPIFFS,
                  entry.iCore);
  }

  file.close();
}



LogEntry Log[LOG_SIZE];
int logIndex = 0;

void logAdd() {
  LogEntry entry;
  entry.pos = logIndex;
  entry.timestamp = millis();
  entry.freeHeap = ESP.getFreeHeap();
  entry.totalSPIFFS = SPIFFS.totalBytes();
  entry.usedSPIFFS = SPIFFS.usedBytes();
  entry.availableSPIFFS = entry.totalSPIFFS - entry.usedSPIFFS;
  entry.iCore = xPortGetCoreID();
  entry.ledStatus = digitalRead(LED_BUILTIN);

  Log[logIndex] = entry;
  logIndex = (logIndex + 1) % LOG_SIZE;

  writeLogEntry(logIndex, entry);
}

void logPrint() {
  Serial.println("[System Log]");
  for (int i = 0; i < LOG_SIZE; i++) {
    int idx = (logIndex + i) % LOG_SIZE;
    Serial.print("Entry ");
    Serial.print(Log[idx].pos);
    Serial.print(": Time=");
    Serial.print(Log[idx].timestamp);
    Serial.print(" ms, FreeHeap=");
    Serial.print(Log[idx].freeHeap);
    Serial.print(" bytes");
    Serial.print("| SPIFFS Total=");
    Serial.print(Log[idx].totalSPIFFS);
    Serial.print(" bytes, Used=");
    Serial.print(Log[idx].usedSPIFFS);
    Serial.print(" bytes, Available=");
    Serial.print(Log[idx].availableSPIFFS);
    Serial.print(" bytes");
    Serial.print(" | Running on Core ");
    Serial.println(Log[idx].iCore);
  }
}

void setup() {
  // Your existing setup code
  delay(5000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  
  Serial.println("[Lab05.ex01] Circular buffer");
  Serial.println();
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");

  if (!SPIFFS.exists(filename)) {
    File file = SPIFFS.open(filename, FILE_WRITE);
    LogEntry empty = {};
    for (int i = 0; i < LOG_SIZE; ++i)
      file.write((const uint8_t*)&empty, sizeof(empty));
    file.close();
  }
}


void loop() {
  delay(2000);
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  logAdd();
  if( ( logIndex % 2 ) == 0 ){
    readLogEntries(filename);
  }
  delay(2000); // Log every 10 seconds
  digitalWrite(LED_BUILTIN, LOW);
}
