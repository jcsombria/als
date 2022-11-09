#pragma once

struct Measurement {
  bool src;
  unsigned long timestamp;
  float y[10];
  int n = 10;
};

class Logger {
	private:
	int buffer_read = 0;
	int buffer_write = 0;
	static const int buffer_size = 10;
	Measurement buffer[buffer_size];
	public:
		void put(const Measurement *point);
		bool get(Measurement&);
		void log(bool isController, unsigned long timestamp, float y[], int n);
		void log_controller(unsigned long timestamp, float y[], int n);
		void log_sensor(unsigned long timestamp, float y[], int n);
		void send(unsigned long timestamp, float y[], int n);
};



