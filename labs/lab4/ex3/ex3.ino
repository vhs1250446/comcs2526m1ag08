#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define LED_BUILTIN 2 // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

const char *ssid = "comcs2526m1ag8"; // Change 99 to your group’s ID
const char *password = "bolofofo123"; // remove if you want the AP to be open.
WiFiServer server(80);

const int LOG_SIZE = 10;
String ledLog[LOG_SIZE];
int logIndex = 0;

void logLedAction(String action)
{
  // Save action to the log
  ledLog[logIndex] = action;
  logIndex = (logIndex + 1) % LOG_SIZE;
}

void printLog()
{
  Serial.println("[LED Action Log]");
  for (int i = 0; i < LOG_SIZE; i++){
    if (ledLog[i].length()) {
      Serial.println(ledLog[i]);
    }
  }
}

void setup() {
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.println();
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  String currentLine;
  WiFiClient client = server.available(); // listen for incoming clients
  if (client) { // if you get a client,
    while (client.connected()) { // loop while the client's connected
      if (client.available()) { // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        if (c == '\n') { // if the byte is a newline character
          // if the current line is blank, that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println(); // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");
            client.print("Click <a href=\"/O\">here</a> to print the log.<br>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') { // if you got anything else but a carriage return character,
            currentLine += c; // add it to the end of the currentLine
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(LED_BUILTIN, HIGH); // GET /H turns the LED on
          logLedAction("LED turned ON");
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW); // GET /L turns the LED off
          logLedAction("LED turned OFF");
        }
        if (currentLine.endsWith("GET /O")) {
          printLog();
        }
      }
    } // close the connection:
    client.stop();
  }
}
