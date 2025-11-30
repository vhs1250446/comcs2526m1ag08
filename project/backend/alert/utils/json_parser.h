/*
 * JSON Parser header
 */

#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include "../alert_server.h"

int parse_sensor_data(const char* json, SensorData* data);

#endif // JSON_PARSER_H
