#include "DHT.h"
#define DHTTYPE DHT11

DHT dht(21, DHTTYPE);
void setup() {
// put your setup code here, to run once:
delay(5000);
Serial.begin(9600);
dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print(h);
  Serial.print(" % - ");
  Serial.print(t);
  Serial.print(" ºC - ");
  Serial.print(dht.computeHeatIndex(t, h, false));
  Serial.println(" ºC"); 
}
