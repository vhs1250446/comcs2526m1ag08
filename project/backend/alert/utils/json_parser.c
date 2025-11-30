/*
 * JSON Parser for smartdata model format
 * Parses incoming sensor data from ESP32 clients
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "../alert.h"
#include "../lib/cJSON.h"

/*
 * Parse smartdata model JSON format
 * Expected format from ESP32:
 * {
 *   "id": "urn:ngsi-ld:WeatherObserved:COMCS-G08-01",
 *   "type": "WeatherObserved",
 *   "dateObserved": "2024-11-20T12:34:56Z",
 *   "temperature": 25.5,
 *   "humidity": 60.0,
 *   "averageTemperature": 24.8,
 *   "averageHumidity": 58.5
 * }
 */
int parse_sensor_data(const char* json, SensorData* data) {
    if (!json || !data) return -1;

    cJSON* root = cJSON_Parse(json);
    if (!root) {
        const char* err = cJSON_GetErrorPtr();
        fprintf(stderr, "cJSON parse error near: %s\n", err ? err : "unknown");
        return -1;
    }

    cJSON* id = cJSON_GetObjectItemCaseSensitive(root, "id");
    if (!cJSON_IsString(id) || (id->valuestring == NULL)) {
        fprintf(stderr, "Failed to parse sensor ID\n");
        cJSON_Delete(root);
        return -1;
    }
    snprintf(data->sensor_id, sizeof(data->sensor_id), "%s", id->valuestring);

    cJSON* type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (cJSON_IsString(type) && type->valuestring) {
        snprintf(data->type, sizeof(data->type), "%s", type->valuestring);
    } else {
        data->type[0] = '\0';
    }

    cJSON* dateObserved = cJSON_GetObjectItemCaseSensitive(root, "dateObserved");
    cJSON* timestamp = cJSON_GetObjectItemCaseSensitive(root, "timestamp");
    const char* ts = NULL;
    if (cJSON_IsString(dateObserved) && dateObserved->valuestring) {
        ts = dateObserved->valuestring;
    } else if (cJSON_IsString(timestamp) && timestamp->valuestring) {
        ts = timestamp->valuestring;
    }
    if (!ts) {
        fprintf(stderr, "Failed to parse timestamp\n");
        cJSON_Delete(root);
        return -1;
    }
    snprintf(data->timestamp, sizeof(data->timestamp), "%s", ts);

    cJSON* temperature = cJSON_GetObjectItemCaseSensitive(root, "temperature");
    cJSON* humidity = cJSON_GetObjectItemCaseSensitive(root, "humidity");
    cJSON* averageTemperature = cJSON_GetObjectItemCaseSensitive(root, "averageTemperature");
    cJSON* averageHumidity = cJSON_GetObjectItemCaseSensitive(root, "averageHumidity");

    data->temperature = cJSON_IsNumber(temperature) ? (float)temperature->valuedouble : 0.0f;
    data->humidity = cJSON_IsNumber(humidity) ? (float)humidity->valuedouble : 0.0f;
    data->avgTemperature = cJSON_IsNumber(averageTemperature) ? (float)averageTemperature->valuedouble : 0.0f;
    data->avgHumidity = cJSON_IsNumber(averageHumidity) ? (float)averageHumidity->valuedouble : 0.0f;

    cJSON_Delete(root);
    return 0;
}
