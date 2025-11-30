/*
 * Test UDP Client for Story 2.4: Reliable UDP Communication
 * 
 * This client tests both QoS policies:
 * - Best effort: Send and forget
 * - Guaranteed delivery: Send and wait for ACK with retries
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <time.h>
#include <math.h>

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 9999
#define BUFFER_SIZE 2048
#define ACK_TIMEOUT_SEC 1
#define MAX_RETRIES 3

// Generate test sensor data in smartdata format
void generate_test_data(char* buffer, size_t size, float temp, float humidity) {
    static int counter = 0;
    time_t now = time(NULL);
    struct tm* tm_info = gmtime(&now);
    char timestamp[64];
    
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", tm_info);
    
    snprintf(buffer, size,
            "{"
            "\"id\":\"urn:ngsi-ld:WeatherObserved:COMCS-G08-TEST-%d\","
            "\"type\":\"WeatherObserved\","
            "\"dateObserved\":\"%s\","
            "\"temperature\":%.2f,"
            "\"humidity\":%.2f,"
            "\"averageTemperature\":%.2f,"
            "\"averageHumidity\":%.2f"
            "}",
            counter++,
            timestamp,
            temp,
            humidity,
            temp - (rand() % 5 - 2),  // Simulate average
            humidity - (rand() % 10 - 5));
}

// Best effort QoS: Send without waiting for ACK
int send_best_effort(int sock, struct sockaddr_in* server_addr, const char* data) {
    printf("\n[Best Effort] Sending data...\n");
    
    int sent = sendto(sock, data, strlen(data), 0,
                     (struct sockaddr*)server_addr, sizeof(*server_addr));
    
    if (sent < 0) {
        perror("sendto failed");
        return -1;
    }
    
    printf("[Best Effort] Sent %d bytes (no ACK expected)\n", sent);
    return 0;
}

// Guaranteed delivery QoS: Send and wait for ACK with retries
int send_guaranteed(int sock, struct sockaddr_in* server_addr, const char* data) {
    char ack_buffer[16];
    struct timeval timeout;
    fd_set readfds;
    int retry;
    
    printf("\n[Guaranteed Delivery] Sending data with ACK...\n");
    
    for (retry = 0; retry < MAX_RETRIES; retry++) {
        if (retry > 0) {
            printf("[Guaranteed Delivery] Retry %d/%d\n", retry, MAX_RETRIES);
        }
        
        // Send data
        int sent = sendto(sock, data, strlen(data), 0,
                         (struct sockaddr*)server_addr, sizeof(*server_addr));
        
        if (sent < 0) {
            perror("sendto failed");
            continue;
        }
        
        // Wait for ACK
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        timeout.tv_sec = ACK_TIMEOUT_SEC;
        timeout.tv_usec = 0;
        
        int result = select(sock + 1, &readfds, NULL, NULL, &timeout);
        
        if (result < 0) {
            perror("select failed");
            continue;
        } else if (result == 0) {
            printf("[Guaranteed Delivery] Timeout waiting for ACK\n");
            continue;
        }
        
        // Receive ACK
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        int recv_len = recvfrom(sock, ack_buffer, sizeof(ack_buffer) - 1, 0,
                               (struct sockaddr*)&from_addr, &from_len);
        
        if (recv_len > 0) {
            ack_buffer[recv_len] = '\0';
            if (strcmp(ack_buffer, "ACK") == 0) {
                printf("[Guaranteed Delivery] ACK received! Success.\n");
                return 0;
            }
        }
    }
    
    printf("[Guaranteed Delivery] Failed after %d retries\n", MAX_RETRIES);
    return -1;
}

int main(int argc, char* argv[]) {
    int sock;
    struct sockaddr_in server_addr;
    char buffer[BUFFER_SIZE];
    int test_mode = 0; // 0=normal, 1=range violations, 2=fluctuations, 3=both
    
    printf("=== UDP Test Client (Story 2.4) ===\n");
    
    if (argc > 1) {
        test_mode = atoi(argv[1]);
    }
    
    printf("Test Mode: ");
    switch(test_mode) {
        case 1: printf("Range Violations\n"); break;
        case 2: printf("Fluctuations\n"); break;
        case 3: printf("Both\n"); break;
        default: printf("Normal Data\n"); break;
    }
    
    // Create socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }
    
    // Setup server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("Invalid address");
        close(sock);
        return 1;
    }
    
    printf("Server: %s:%d\n\n", SERVER_IP, SERVER_PORT);
    
    // Test data generation based on mode
    float temp, humidity;
    
    switch(test_mode) {
        case 1: // Range violations (Story 2.2)
            printf("=== Test Case: Range Violations ===\n");
            
            // Temperature out of range (>50°C)
            generate_test_data(buffer, sizeof(buffer), 55.0, 50.0);
            printf("Test 1: Temperature > 50°C\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            sleep(1);
            
            // Humidity out of range (<20%)
            generate_test_data(buffer, sizeof(buffer), 25.0, 15.0);
            printf("\nTest 2: Humidity < 20%%\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            sleep(1);
            
            // Both out of range
            generate_test_data(buffer, sizeof(buffer), -5.0, 85.0);
            printf("\nTest 3: Both out of range\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            break;
            
        case 2: // Fluctuations (Story 2.3)
            printf("=== Test Case: Fluctuations ===\n");
            
            // Large temperature fluctuation
            temp = 25.0;
            humidity = 50.0;
            snprintf(buffer, sizeof(buffer),
                    "{"
                    "\"id\":\"urn:ngsi-ld:WeatherObserved:COMCS-G08-FLUCT\","
                    "\"type\":\"WeatherObserved\","
                    "\"dateObserved\":\"2024-11-20T12:00:00Z\","
                    "\"temperature\":%.2f,"
                    "\"humidity\":%.2f,"
                    "\"averageTemperature\":%.2f,"
                    "\"averageHumidity\":%.2f"
                    "}",
                    temp, humidity, temp - 5.0, humidity);
            printf("Test 1: Temp diff = 5°C (threshold 2°C)\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            sleep(1);
            
            // Large humidity fluctuation
            snprintf(buffer, sizeof(buffer),
                    "{"
                    "\"id\":\"urn:ngsi-ld:WeatherObserved:COMCS-G08-FLUCT\","
                    "\"type\":\"WeatherObserved\","
                    "\"dateObserved\":\"2024-11-20T12:01:00Z\","
                    "\"temperature\":%.2f,"
                    "\"humidity\":%.2f,"
                    "\"averageTemperature\":%.2f,"
                    "\"averageHumidity\":%.2f"
                    "}",
                    temp, humidity, temp, humidity - 10.0);
            printf("\nTest 2: Humidity diff = 10%% (threshold 5%%)\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            break;
            
        case 3: // Combined test
            printf("=== Test Case: Combined (QoS + Validation) ===\n");
            
            // Normal data with best effort
            generate_test_data(buffer, sizeof(buffer), 22.0, 55.0);
            printf("Test 1: Normal data (best effort)\n%s\n", buffer);
            send_best_effort(sock, &server_addr, buffer);
            sleep(1);
            
            // Out of range with guaranteed
            generate_test_data(buffer, sizeof(buffer), 60.0, 90.0);
            printf("\nTest 2: Out of range (guaranteed)\n%s\n", buffer);
            send_guaranteed(sock, &server_addr, buffer);
            break;
            
        default: // Normal operation
            printf("=== Test Case: Normal Operation ===\n");
            for (int i = 0; i < 5; i++) {
                temp = 20.0 + (rand() % 10);
                humidity = 40.0 + (rand() % 20);
                
                generate_test_data(buffer, sizeof(buffer), temp, humidity);
                printf("\nTest %d:\n%s\n", i + 1, buffer);
                
                if (i % 2 == 0) {
                    send_guaranteed(sock, &server_addr, buffer);
                } else {
                    send_best_effort(sock, &server_addr, buffer);
                }
                
                sleep(2);
            }
            break;
    }
    
    close(sock);
    printf("\n=== Test Complete ===\n");
    return 0;
}
