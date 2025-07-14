/*
 * httpserver.c
 *
 *  Created on: Jul 10, 2025
 *      Author: boran
 *
 *
 */
#include "httpserver.h"

// HTML template for the webpage
const char html_template[] =
"<!DOCTYPE html>\r\n"
"<html>\r\n"
"<head>\r\n"
"<title>STM32 Sensor Monitor</title>\r\n"
"<meta charset='UTF-8'>\r\n"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>\r\n"
"<style>\r\n"
"body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }\r\n"
".container { max-width: 600px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }\r\n"
"h1 { color: #333; text-align: center; }\r\n"
".sensor-card { background-color: #e3f2fd; padding: 15px; margin: 10px 0; border-radius: 5px; border-left: 4px solid #2196f3; }\r\n"
".sensor-value { font-size: 24px; font-weight: bold; color: #1976d2; }\r\n"
".sensor-unit { font-size: 14px; color: #666; }\r\n"
".timestamp { color: #999; font-size: 12px; text-align: right; }\r\n"
".refresh-btn { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin: 10px auto; display: block; }\r\n"
".refresh-btn:hover { background-color: #45a049; }\r\n"
"</style>\r\n"
"<script>\r\n"
"function refreshData() {\r\n"
"    location.reload();\r\n"
"}\r\n"
"setInterval(refreshData, 10000);\r\n"
"</script>\r\n"
"</head>\r\n"
"<body>\r\n"
"<div class='container'>\r\n"
"<h1>STM32 Environmental Monitor</h1>\r\n"
"<div class='sensor-card'>\r\n"
"<h3>Temperature</h3>\r\n"
"<span class='sensor-value'>%.1f</span><span class='sensor-unit'>°C</span>\r\n"
"</div>\r\n"
"<div class='sensor-card'>\r\n"
"<h3>Pressure</h3>\r\n"
"<span class='sensor-value'>%ld</span><span class='sensor-unit'>Pa</span>\r\n"
"</div>\r\n"
"<div class='timestamp'>Last updated: %lu seconds ago</div>\r\n"
"<button class='refresh-btn' onclick='refreshData()'>Refresh Data</button>\r\n"
"</div>\r\n"
"</body>\r\n"
"</html>\r\n";


const char http_404[] =
		"HTTP/1.1 404 Not Found\r\n"
		"Content-Type: text/html\r\n"
		"Content-Length: 48\r\n"
		"Connection: close\r\n"
		"\r\n"
		"<html><body><h1>404 Not Found</h1></body></html>";
// ~87 bytes
const char* http_response =
"HTTP/1.1 200 OK\r\n"
"Content-Type: text/html\r\n"
"Connection: close\r\n"
"Content-Length: %d\r\n"
"\r\n"
"%s";

// length ~231
const char* html =
"<html><head><title>Sensor Readings</title></head>"
"<body style=\"font-family: sans-serif;\">"
"<h1>Sensor Data</h1>"
"<p><strong>Temperature:</strong> %.2f &deg;C</p>"
"<p><strong>Pressure:</strong> %ld Pa</p>"
"</body></html>";

char body[2048];

int getResHttp(char *req, char *res, SensorData_t *sensor_data){
	int body_len, total_len;
	//print2sh(req);
	uint32_t current_time = HAL_GetTick() / 1000;
	if (strncmp(req, "GET / ", 6) == 0){
		body_len = snprintf(body, 2048, html_template, sensor_data->temperature, sensor_data->pressure, current_time - sensor_data->timestamp);
		total_len = snprintf(res, 2048, http_response, body_len, body);
	}
	else{
		total_len = snprintf(res, 256, http_404);
	}
	return total_len;
}
