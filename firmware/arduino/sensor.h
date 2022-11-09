#include <TimerOne.h>
#include <ArduinoJson.h>
#include "logger.h"
#include "pid.h"

#define SENSOR_4_30

#ifdef SENSOR_4_30
//	const float v[] = {3.0f, 2.3f, 1.6f, 1.00f, 0.53f, 0.31f, 0.2f, 0.1f};
  const float v[] = {1.7f, 1.55f, 1.0f, 0.9f, 0.53f, 0.36f, 0.26f, 0.2f};
#else
	// Air Levitation System V1
	const float v[] = {2.3f, 1.65f, 1.4f, 1.3f, 1.05f, 0.85f, 0.65f, 0.5f};
	// Air Levitation System V2
	//	static const float v[] = {2.45f, 1.9f, 1.8f, 1.61f, 1.41f, 1.15f, 0.94f, 0.89f};
#endif

// Sensor calibration
float lookup(float voltage) {
	static const float c = 1.0f;
	static const int v_length = 8;
	static const float h = 35.0f / (v_length - 1), v_min = c*v[v_length-1], v_max = c*v[0];
	float v_sat = sat(voltage, v_min, v_max);
	int i = 0;
	while(v_sat < v[i] && i < v_length) {
		i++;
	};
	if(i == 0) {
		return 0;
	}
	float offset = (v_sat - v[i-1])/ (v[i] - v[i-1]);
	return h*(i-1 + offset);
}

// Mio
//const float a = 34.6695, b = -1.3847;
//const float a = 30.481793, b = -1.007660;
const float a = 63.71, b = -0.7309;

// ALS-001
float exp_fit(float voltage) {
  return a*exp(b*voltage);
}
