#pragma once

float sat(float value, float min, float max) {
  if(value < min) {
  return min;
  }

  if(value > max) {
  return max;
  }
  return value;
}

class Controller {
  protected:
    float sp = 0.0;
    float *params;
    float *state;
  public:
    virtual float update(float y);
    void setpoint(float sp);
    float getSetpoint();
    const float* getState();
    const float* getParams();
};

class Component {
  protected:
    int pin = A0;
  public:
    void setPin(int pin);
};

class Sensor : public Component {
  private:
    static const int points = 5;
    float y_cum = 0;
    float y_last[points];
    int i_last = 0;
    float (*f)(float voltage);
  public:
    Sensor();
    void update();
    float mean();
    float read();
    float convert(float voltage);
    void setConvertFunction(float (*f)(float));
};

class Actuator : public Component {
  private:
    float min = 0.0;
    float max = 1.0;
  public:
    float write(float value);
    void setRange(float min, float max);
    void setMin(float min);
    void setMax(float max);
};
