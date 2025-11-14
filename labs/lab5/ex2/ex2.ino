#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "f9892748-ae17-47c8-a2fe-84703bca3fcf"
#define CHARACTERISTIC_UUID "b89bb704-1ce9-4b0c-afde-58d0caa9315c"

BLEServer* pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

void setup() {
  delay(5000);

  Serial.begin(9600);

  Serial.println("Starting BLE...");
  BLEDevice::init("comcs2425mlag8");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                                                   BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setValue("Hello, World"); // can be a sensor reading
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising(); // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // put your main code here, to run repeatedly:
}
