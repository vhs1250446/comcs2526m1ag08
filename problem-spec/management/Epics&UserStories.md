This document translates the project requirements from Use\_Case\_v2526\_en.pdf into a User Story format, grouped by component.

### **Project Epics**

* **Epic 1: Real-time Monitoring:** Provide a live view of the warehouse's environmental status.  
* **Epic 2: Proactive Alerting:** Automatically detect and report on environmental anomalies and sensor failures.  
* **Epic 3: Data Resilience:** Ensure no data is lost during network outages or device restarts.

## **1\. Monitoring Clients (ESP32 & Raspberry Pi Pico)**

Actor: System (as a data provider for the backend)  
Hardware: ESP32, Raspberry Pi Pico, DHT11 Sensors

### **Story 1.1: Sensor Data Acquisition**

As a system, I want the monitoring clients to read the temperature and humidity sensors every second so that the data provided to the backend is always up-to-date.  
Acceptance Criteria:

* (EO 1.b, 3.a) One ESP32 and one Raspberry Pi Pico are used.  
* (EO 1.b, 3.a) Each device is equipped with a DHT11 sensor.  
* (EO 3.a) Sensor readings (temperature and humidity) are taken once every second.

### **Story 1.2: Central Data Publication**

As a system, I want the monitoring clients to publish their sensor data via MQTT so that the Command Centre can subscribe to and display the data.  
Acceptance Criteria:

* (EO 3.b) Data is sent via the MQTT protocol.  
* (EO 3.c, AC 3.a) The message payload must be in the smartdata model format.  
* (EO 1.c) This communication is established with the Command Centre (which subscribes to the topic).

### **Story 1.3: Alert Server Data Transmission**

As a system, I want the monitoring clients to send their sensor data via UDP to the Alert Server so that advanced analysis and anomaly detection can be performed.  
Acceptance Criteria:

* (EO 3.b) Data is sent via the UDP protocol.  
* (EO 2.d) The data package must include both the *current* sensor value and the *average* value (Note: client must calculate its own running average).  
* (EO 3.c, AC 3.a) The message payload must be in the smartdata model format.

### **Story 1.4: Offline Data Buffering**

As a system administrator, I want the monitoring clients to detect communication failures and store sensor readings locally so that no telemetry data is lost during a network outage.  
Acceptance Criteria:

* (AC 3.c) Communication error handling is implemented.  
* (EO 4.a, AC 3.b) In case of a failure, a "rolling window" (circular buffer) is used.  
* (EO 4.a, AC 3.b) This rolling window must be implemented using a file in the SPIFFS area.

### **Story 1.5: Offline Data Recovery**

As a system administrator, I want the monitoring clients to automatically send all stored data from the rolling window when communication is re-established so that the central system is backfilled with any missing data.  
Acceptance Criteria:

* (EO 4.a) When communication is restored, the client sends all data from the SPIFFS file to both the UDP server and the Command Centre (via MQTT).  
* (EO 4.c) In the event of a device restart, the client must check SPIFFS and transmit any pending data from a previous session.

## **2\. Alert Server (UDP Server)**

Actor: Quality Control (QC) Manager (as the recipient of alerts)  
Hardware: Virtual Machine

### **Story 2.1: Concurrent Data Ingestion**

As a system, I want the UDP server to be able to receive data from multiple monitoring clients concurrently so that the system can scale as more sensors are added.  
Acceptance Criteria:

* (EO 2.a, AC 2.a) The server is implemented as a UDP server.  
* (EO 2.c, AC 2.c) The server can successfully receive and process data from multiple clients at the same time.

### **Story 2.2: Data Range Validation**

As a QC Manager, I want the Alert Server to validate that all incoming sensor data is within the equipment's specified operational range so that I can be alerted if conditions are outside safe limits.  
Acceptance Criteria:

* (EO 2.d) The server validates temperature readings are between 0°C and 50°C.  
* (EO 2.d) The server validates humidity readings are between 20% and 80%.  
* (EO 2.e) An alert is generated (to log/screen) if a reading is outside this range.  
* (EO 2.f) This alert is sent via MQTT to the Command Centre.

### **Story 2.3: Data Fluctuation Analysis**

As a QC Manager, I want the Alert Server to calculate the difference between a sensor's current reading and its average reading so that I can be alerted to rapid, problematic fluctuations.  
Acceptance Criteria:

* (EO 2.d, AC 2.e) The server receives both the *current* and *average* values from the client.  
* (EO 2.d, AC 2.e) The server calculates the differential between these two values.  
* (EO 2.e) An alert is generated (to log/screen) if the temperature differential exceeds \~2°C.  
* (EO 2.e) An alert is generated (to log/screen) if the humidity differential exceeds \~5%.  
* (EO 2.f, AC 2.f) This alert is sent via MQTT to the Command Centre.

### **Story 2.4: Reliable UDP Communication**

As a system administrator, I want the UDP communication to support both "best effort" and "guaranteed delivery" policies so that I can ensure critical data is not lost in transit.  
Acceptance Criteria:

* (EO 2.b, AC 2.b) Two QoS policies (best effort, guaranteed delivery) are implemented for UDP.  
* (EO 2.b.i, AC 2.d) The developer must provide tests that prove the guaranteed delivery protocol works correctly (e.g., by simulating packet loss).

## **3\. Command Centre (Node-RED)**

Actor: Warehouse Manager / QC Manager (as the end-user)  
Hardware: Virtual Machine

### **Story 3.1: Central Monitoring Dashboard**

As a Warehouse Manager, I want a central command centre dashboard so that I can monitor the environmental status of a specific work cell from one place.  
Acceptance Criteria:

* (EO 1.a, AC 1.a) The command centre is implemented using Node-RED.

### **Story 3.2: Live Environmental Display**

As a Warehouse Manager, I want to see the real-time average temperature and humidity values so that I can quickly assess the current state of the warehouse.  
Acceptance Criteria:

* (EO 1.c) The dashboard subscribes to data via the MQTT protocol.  
* (EO 1.d, AC 1.b) The dashboard displays the *average* temperature and humidity.  
* (AC 1.b) The dashboard also displays *instant* values.  
* (AC 1.d) The dashboard correctly parses the smartdata model format.

### **Story 3.3: Historical Data Visualization**

As a QC Manager, I want to see historical graphs of temperature and humidity so that I can analyze trends and identify when problems began.  
Acceptance Criteria:

* (AC 1.b) The dashboard displays historical graphs for both temperature and humidity.

### **Story 3.4: Visual Alert Notification**

As a Warehouse Manager, I want to see a clear visual indicator for alerts on my dashboard so that I am immediately notified when an environmental breach occurs.  
Acceptance Criteria:

* (EO 1.e, AC 1.c) The dashboard includes an "Alert indicator."  
* (EO 1.e) The indicator activates when alerts are received (e.g., temp \> 20°C, humidity \< 10%).  
* (AC 2.f) The alerts are received via MQTT from the Alert Server.

## **4\. Bonus Stories (Additional Valuation)**

### **Story 4.1: BLE Redundancy**

As a system administrator, I want the system to use BLE as a redundant communication channel so that monitoring can continue even if the primary network (Wi-Fi) fails.  
Acceptance Criteria:

* (Add. Crit. 1\) BLE is used as a fallback communication path.

### **Story 4.2: Component Failure Detection**

As a system administrator, I want the system to detect if a specific component (like a sensor or server) has failed so that I can dispatch maintenance.  
Acceptance Criteria:

* (Add. Crit. 3\) A failure detection mechanism is implemented (e.g., a "heartbeat" from clients, or the Alert Server reporting a client has gone silent).