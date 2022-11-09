// -- Controller --
void Controller::setpoint(float sp) {
  this->sp = sp;
}

float Controller::getSetpoint() {
  return this->sp;
}

const float* Controller::getParams() {
  return params;
}

const float* Controller::getState() {
  return state;
}

void Controller::setActuator(Actuator *actuator) {
  this->actuator = actuator;
}

Actuator* Controller::getActuator() {
  return this->actuator;
}

// -- Component --
void Component::setPin(int pin) {
  this->pin = pin;
}

// -- Sensor --
Sensor::Sensor() {
  for(int i=0; i<points; i++) {
    y_last[i] = 0;
  }
}

float Sensor::read() {
	return y_last[i_last];
}

void Sensor::update() {
  int sensorReading = analogRead(pin);
  float voltage = sensorReading * 5.0 / 1024.0;
  y_last[i_last] = voltage;
  i_last = (i_last + 1) % points;
}

float Sensor::mean() {
  y_cum = 0;
  for(int i=0; i<points; i++) {
    y_cum += y_last[i];
  }
  return y_cum / (float)points;
}

float Sensor::convert(float voltage) {
  return f(voltage);
}

void Sensor::setConvertFunction(float (*f)(float)) {
  this->f = f;
}

// -- Actuator --
float Actuator::write(float value) {
  float u = sat(value);
  analogWrite(pin, 255*u);
  return u;
}

void Actuator::setRange(float min, float max) {
  if(max < min) {
    return;
  }
  this->setMin(min);
  this->setMax(max);
}

void Actuator::setMin(float min) {
  if(min > this->max) {
    return;
  }
  this->min = (min >= 0) && (min <= 1) ? min : this->min;
}

void Actuator::setMax(float max) {
  if(max < this->min) {
    return;
  }
  this->max = (max >= 0) && (max <= 1) ? max : this->max;
}

float Actuator::sat(float u) {
  if(u < min) { return min; }
  if(u > max) { return max; }
  return u;
}


