#include <WiFi.h>
#include <ESPAsyncWebServer.h> // Include the AsyncWebServer library

#define LED_BUILTIN 2 // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

const char *ssid = "comcs2526m1ag8"; // Change 99 to your group’s ID
const char *password = "bolofofo123"; // remove if you want the AP to be open.
AsyncWebServer server(80);
const int LOG_SIZE = 10;
String ledLog[LOG_SIZE];
int logIndex = 0;

void logLedAction(String action) {
  // Save action to the log
  ledLog[logIndex] = action;
  logIndex = (logIndex + 1) % LOG_SIZE;
  // Print the log to Serial Monitor
  Serial.println("[LED Action Log]");
  for (int i = 0; i < LOG_SIZE; i++) {
    int idx = (logIndex + i) % LOG_SIZE;
    if (ledLog[idx].length() > 0) {
      Serial.println(ledLog[idx]);
    }
  }
}

void setup() {
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("[Lab04.ex04] AP with mitiple clients");
  Serial.println();
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  // Serve the main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<h1>ESP32 LED Control</h1>\
<button onclick=\"location.href='/on'\">Turn ON LED</button>\
<button onclick=\"location.href='/off'\">Turn OFF LED</button>";
      request->send(200, "text/html", html);
    });
  // Turn LED on
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
      digitalWrite(LED_BUILTIN, HIGH);
      logLedAction("LED ON at " + String(millis()) + " ms");
      request->redirect("/"); // Redirect back to main page
    });
  // Turn LED off
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
      digitalWrite(LED_BUILTIN, LOW);
      logLedAction("LED Off at " + String(millis()) + " ms");
      request->redirect("/"); // Redirect back to main page
    });
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // No need to poll clients!
}
