#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "DHT.h"
#include <ArduinoJson.h>

#define DHTTYPE DHT11

// DHT sensor setting
DHT dht(21, DHTTYPE);

//---- WiFi settings
const char* ssid = "labs";
const char* password = "782edcwq#";

//---- MQTT Broker settings
const char* mqtt_server = "2f0de4ef7266438a8f0a43350b1fa391.s1.eu.hivemq.cloud"; // replace with your broker url
const char* mqtt_username = "comcs2526g08"; //replace with your broker id
const char* mqtt_password = "Bolofofo123";//replace with your broker password
const int mqtt_port =8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE (50)

char msg[MSG_BUFFER_SIZE];

char temperatureData[100];
char humidityData[100];
char heatIndexData[100];

const char *temperatureTopic = "/comcs/g08/temperature";
const char *humidityTopic = "/comcs/g08/humidity";
const char *heatIndexTopic = "/comcs/g08/heatIndex";


void setup() {
  delay(5000);
  
  Serial.begin(9600);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());

  while (!Serial) delay(1);

  espClient.setInsecure(); // !! espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Beginning to listen for DHT sensor values.");
  dht.begin();
}

String serializeToJson(String key, float value)
{
  JsonDocument doc;
  doc[key] = value;

  String output;
  serializeJson(doc, output);

  return output;
}

void loop() {
  if (!client.connected())
    reconnect();

  client.loop(); 
  
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float heatIndex = dht.computeHeatIndex(temperature, humidity, false);

  sprintf(temperatureData,"%f", temperature);
  String temperatureJson = serializeToJson("Temperature", temperature);
  Serial.println(temperatureJson);

  sprintf(humidityData,"%f", humidity);
  String humidityJson = serializeToJson("Humidity", humidity);
  Serial.println(humidityJson);

  sprintf(heatIndexData,"%f", heatIndex);
  String heatIndexJson = serializeToJson("HeatIndex", heatIndex);
  Serial.println(heatIndexJson);

  
  //Serial.print("Temperature - ");
  //Serial.println(temperatureData);

  
  //Serial.print("Humidity - ");
  //Serial.println(humidityData);

  
  //Serial.print("HeatIndex - ");
  //Serial.println(heatIndexData);
  
  // publishMessage(temperatureTopic, String(temperatureData), true);
  publishMessage(temperatureTopic, temperatureJson, true);
  publishMessage(humidityTopic, humidityJson, true);
  publishMessage(heatIndexTopic, heatIndexJson, true);
  
  delay(5000);
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    String clientId = "ESP32-G08-"; // change the groupID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(temperatureTopic);
      client.subscribe(humidityTopic);
      client.subscribe(heatIndexTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds"); // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//=======================================
// This callback is called every time we have a message from the broker
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Callback - ");
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}

//======================================= publising as string
void publishMessage(const char* topic, String payload , boolean retained){
if (client.publish(topic, payload.c_str(), true))
  Serial.println("Message published ["+String(topic)+"]: "+payload);
}
