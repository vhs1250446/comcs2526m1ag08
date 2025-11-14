#include <ArduinoJson.h>

void setup() {
    // put your setup code here, to run once:
  delay(2000);
  Serial.begin(9600);

  Serial.println("[Lab08.ex01]");
  // Step 1: Create a JsonDocument
  JsonDocument doc;
  // Step 2: Add data to the JSON object
  doc["device"] = "ESP32";
  doc["temperature"] = 25.6;
  doc["humidity"] = 55;
  // Step 3: Serialize the JSON to a string
  String output;
  serializeJson(doc, output);
  // Print the JSON string
  Serial.print("Serialized JSON: ");
  Serial.println(output);

  //Deserialize
  // Step 1: Create a JsonDocument
  JsonDocument doc2;
  // Step 2: Parse the JSON string
  DeserializationError error = deserializeJson(doc2, output);
  // Check if parsing was successful
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
  // Step 3: Extract values
  float temperature = doc2["temperature"];
  int humidity = doc2["humidity"];
  // Print the values
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
}

void loop() {
  // put your main code here, to run repeatedly:

}
