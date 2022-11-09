#include <ServoTimer2.h>

unsigned long currentMillis = 0;
unsigned long lastMillis = 0;
boolean tic_flag = false;

SIGNAL(TIMER0_COMPA_vect) {
  currentMillis = millis();
  unsigned long elapsed = currentMillis - lastMillis;
  if(elapsed >= 20) {
    lastMillis = currentMillis;
    tic_flag = true;
  }
}

// Serial Comm
#include <ArduinoJson.h>
#include "logger.h"

#define BAUDRATE 115200

ServoTimer2 servo;

String inputString = "";
boolean stringComplete = false;
Logger logger;

struct Data {
  bool sensor;
  float setpoint;
  float params[5];
  bool setpointChanged = false;
  bool paramsChanged = false;
} data;

// Controller

#define USE_PID
#include "sliding.h"
#include "opt2.h"
#include "pid.h"

#if defined(USE_SLIDING)
  Sliding controller;
#elif defined(USE_OPT2)
  OPT2 controller;
#elif defined(USE_PICI)
  PICI controller;
#elif defined(USE_PI2D)
  PI2D controller;
#elif defined(USE_PIARH)
  PIARH controller;
#else
  PID controller;
#endif

// Actuator
#define FAN_MIN 0.0
#define FAN_MAX 1.0
Actuator fan;

// Sensor
#define SENSOR_CONVERT_FUNCTION exp_fit
#include "sensor.h"

const uint16_t period = 20000;
uint8_t sensor_oversampling = 20000 / period;
uint8_t controller_oversampling = 100000 / period;
uint8_t iter_sensor = 0;
uint8_t iter_controller = 0;
Sensor sensor;

// Pin mapping
const int pirPin = A5;
const int fanPin = 9;
const int fanspeedPin = 2;
const int servoPin = 3;

int servo_state = 0;
int servo_angle = 45;
int dist_angle = 30;
float y_voltage_last = 0;
float y_last = 0;
float u_extern = 0;
boolean tracking = false;

// Control Loop
void tic() {
  // Sensor
  sensor.update();
  if((++iter_sensor) >= sensor_oversampling) {
    iter_sensor = 0;
    y_voltage_last = sensor.mean();
    y_last = sensor.convert(y_voltage_last);
  }

  // Controller
  if((++iter_controller) >= controller_oversampling) {
    iter_controller = 0;
    float u = tracking ? u_extern : controller.update(y_last);
    float actual_u = fan.write(u);
    const float *state = controller.getState();
    float point_controller[] = {
      controller.getSetpoint(),
      y_voltage_last,
      y_last,
      u,
      actual_u,
      state[0],
      state[1],
      state[2],
    };
    int n = sizeof(point_controller) / sizeof(float);
    logger.log_controller(millis(), point_controller, n);
  }

  // Disturbance
  if(servo_state == 0 || servo_state == 50) {
    servo_state = 0;
  } else {
    servo_state ++;
  }
}

void setup(void) {
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  Serial.begin(BAUDRATE);
  inputString.reserve(200);
  servo.attach(servoPin);
  servo.write(90);
  pinMode(fanPin, OUTPUT);
  pinMode(fanspeedPin, INPUT);
  sensor.setPin(pirPin);
  sensor.setConvertFunction(SENSOR_CONVERT_FUNCTION);
  fan.setPin(fanPin);
  fan.setRange(FAN_MIN, FAN_MAX);
  controller.setActuator(&fan);
  controller.setpoint(20);
}

#define SERVO_MIN 750
#define SERVO_MAX 1650
int angle2duty(float angle) {
  int duty = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 90;
  if(duty < SERVO_MIN) {
    duty = SERVO_MIN;
  } else if (duty > SERVO_MAX) {
    duty = SERVO_MAX;
  }
  return duty;
}
  
void loop(void) {
  if(tic_flag) {
    tic_flag = false;
    tic();
  }
  
  Measurement point;
  if(logger.get(point)) {
    sendData(point);
  }

  if(stringComplete) {
    int len = inputString.length();
    char buf[200];
    inputString.toCharArray(buf, len);
    deserialize(data, buf);
    if(data.setpointChanged) {
      data.setpointChanged = false;
      controller.setpoint(data.setpoint);
    }
    if(data.paramsChanged) {
      data.paramsChanged = false;
      controller.setParams(data.params);
    }
    inputString = "";
    stringComplete = false;
  }
//  if(servo_state == 1) {
//    servo.write(angle2duty(servo_angle + dist_angle));
//  } else if(servo_state == 50) {
//    servo.write(angle2duty(servo_angle));
//  }
}

bool deserialize(struct Data& data, char* json) {
  StaticJsonBuffer<150> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json);
  int n, i;
  if(root.containsKey("setpoint")) {
    data.setpoint = root["setpoint"];
    data.setpointChanged = true;
  }
  if(root.containsKey("params")) {
    n = root["params"].size();
    noInterrupts();
    for(i=0; i<n; i++) {
      data.params[i] = root["params"][i];
    }
    interrupts();
    data.paramsChanged = true;
  }
  if(root.containsKey("servo")) {
    servo_angle = (int)root["servo"];
    servo.write(angle2duty(servo_angle));
  }

  if(root.containsKey("dist")) {
    dist_angle = constrain((int)root["dist"], -30, 30);
    servo_state = 1;
  }

  if(root.containsKey("u")) {
    u_extern = (double)root["u"];
  }

  if(root.containsKey("extern")) {
    tracking = (bool)root["extern"];
  }
//  if(root.containsKey("fan")) {
//    double fan_min = root["fan"][0],
//           fan_max = root["fan"][1];
//    fan.setRange(fan_min, fan_max);
//    servo_state = 1;
//  }

  if(root.containsKey("Tsensor")) {
    long Tsensor = (long)root["Tsensor"];
    sensor_oversampling = Tsensor / period;
  }

  return root.success();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
