#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "driver/gpio.h"
#include "driver/twai.h"

#define LED_BUILTIN 2
#define SERVICE_UUID "0000181A-0000-1000-8000-00805F9B34FB"
#define SENSOR_CHARACTERISTIC_UUID "00002A6E-0000-1000-8000-00805F9B34FB"

union Temperature_converter{
 uint32_t value;
 float f2;
 int16_t tValue;
};

Temperature_converter converter;

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

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
    /*String svalue = pLedCharacteristic->getValue();
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
    }*/
  }
};

void setup_ble()
{
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
  /*pLedCharacteristic = pService->createCharacteristic(
                                                      LED_CHARACTERISTIC_UUID,
                                                      BLECharacteristic::PROPERTY_WRITE);
  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());*/
  //https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  //pLedCharacteristic->addDescriptor(new BLE2902());
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

void setup_twai_bus()
{
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4,
  TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.printf("Driver installed\n");
  } else {
    Serial.printf("Failed to install driver\n");
  }
  //Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.printf("Driver started\n");
  } else {
    Serial.printf("Failed to start driver\n");
  }
}

void setup() {
  delay(5000);
  Serial.begin(9600);

  setup_ble();
  setup_twai_bus();
}

void loop() {

  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
    Serial.printf("Message received ");
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  } else {
    Serial.printf("Failed to receive message\n");
    return;
  }
  //Process received message
  if (message.extd) {
    Serial.printf("Message is in Extended Format");
  } else {
    Serial.printf("Message is in Standard Format");
  }

  Serial.printf(" ID is 0x%lX", message.identifier);

  if (!(message.rtr)) {
  for (int i = 0; i < message.data_length_code; i++) {
    converter.value |= (message.data[i] << (i*8));
    Serial.printf("Data byte %d = %d\n", i, message.data[i]);
  }
    Serial.printf(" data %f", converter.f2);
    Serial.println("");
  }
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW

  // notify changed value
  if (deviceConnected) {
    
    pSensorCharacteristic->setValue((uint8_t*)&converter.tValue * 100, 2);
    pSensorCharacteristic->notify();
    Serial.print("New value notified: ");
    Serial.println(converter.tValue);

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
