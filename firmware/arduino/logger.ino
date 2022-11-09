#include "logger.h"
#include <ArduinoJson.h>

#define TIMESTAMP "timestamp"
#define OUTPUT "y"

void sendData(const struct Measurement& point) {
  StaticJsonBuffer<100> jsonBuffer;
	JsonObject& root = jsonBuffer.createObject();
	root[TIMESTAMP] = point.timestamp;
	JsonArray& y = root.createNestedArray(OUTPUT);
	if(point.src) {
    root["src"] = "controller";
	} else {
    root["src"] = "sensor";
  }
	for(int i=0; i<point.n; i++) { y.add(point.y[i], 6); }
	root.printTo(Serial);
	Serial.println();
}

void Logger::put(const Measurement *point) {
	buffer[buffer_write] = *point;
	buffer_write = (buffer_write + 1) % buffer_size;
}

bool Logger::get(Measurement& point) {
	bool empty = (buffer_read == buffer_write);
	if(!empty) {
		point = buffer[buffer_read];
		buffer_read = (buffer_read + 1) % buffer_size;
	}
	return !empty;
}

void Logger::log(bool isController, unsigned long timestamp, float y[], int n) {
  Measurement point;
  point.src = isController;
  point.timestamp = timestamp;
  point.n = n;
  for(int i=0; i<point.n; i++) { point.y[i] = y[i]; }
  put(&point);
}


void Logger::log_sensor(unsigned long timestamp, float y[], int n) {
  log(false, timestamp, y, n);
}

void Logger::log_controller(unsigned long timestamp, float y[], int n) {
  log(true, timestamp, y, n);
}

void Logger::send(unsigned long timestamp, float y[], int n) {
	Measurement point;
	point.timestamp = timestamp;
	point.n = n;
	for(int i=0; i<point.n; i++) { point.y[i] = y[i]; }
	sendData(point);
}
