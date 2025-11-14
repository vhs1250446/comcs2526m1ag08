#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #define SERVICE_UUID "460656f9-825f-4fa0-9465-59057991c6b6"

#define SERVICE_UUID "0000181A-0000-1000-8000-00805F9B34FB"
// SENSOR_CHARACTERISTIC_UUID (0x2A6E): Temperature Characteristic
#define SENSOR_CHARACTERISTIC_UUID "00002A6E-0000-1000-8000-00805F9B34FB"
// #define SENSOR_CHARACTERISTIC_UUID "25253096-f508-4d69-8914-190987407d12"

#define LED_CHARACTERISTIC_UUID "f95cfcab-1cca-4e3b-b8a5-7a49d63a88fd"

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
const int ledPin = 2; // Use the appropriate GPIO pin for your setup

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    String svalue = pLedCharacteristic->getValue();
    Serial.print("Characteristic event, written: ");
    Serial.println(svalue);
    if (svalue.length() > 0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(svalue[0])); // Print the integer value
        int receivedValue = static_cast<int>(svalue[0]);
      if (receivedValue == 1) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
        value = 0;
      }
    }
  }
};

void setup() {
  delay(5000);
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  // Create the BLE Device
  BLEDevice::init("comcs2324g8");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                                                         SENSOR_CHARACTERISTIC_UUID,
                                                         BLECharacteristic::PROPERTY_READ |
                                                         BLECharacteristic::PROPERTY_WRITE |
                                                         BLECharacteristic::PROPERTY_NOTIFY |
                                                         BLECharacteristic::PROPERTY_INDICATE
                                                         );
  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                                                      LED_CHARACTERISTIC_UUID,
                                                      BLECharacteristic::PROPERTY_WRITE);
  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  //https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("New value notified: ");
    Serial.println(value);

    if (value == 20) {
      digitalWrite(ledPin, HIGH);

      delay(5000);
      digitalWrite(ledPin, LOW);
      value = 0;
    }

    delay(3000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}
