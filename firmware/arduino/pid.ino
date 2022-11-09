#ifdef USE_PID
// -- PID Controller --
PID::PID() {
  char* params_names[7] = {"kp", "ki", "kd", "u0", "N", "a", "b"};
  char* state_names[4] = {"I", "D", "e_prev", "y_prev"};
  params = new float[7] {0.005, 0.0005, 0.00, 0.62};
  state = new float[4] {0.0, 0.0, 0.0, 0.0};
}

float PID::update(float y) {
  const float kp = params[0], ki = params[1], kd = params[2], u0 = params[3], N = params[4], a = params[5], b = params[6];
  float I = state[0], D = state[1], e_prev = state[2], y_prev = state[3];

  // Compute the control action
  float e = sp - y;
  float P = kp*e;
  float I_prev = I;
  I += ki*e*period;
  D = a*D + b*(y - y_prev);

  float v = u0 + P + I + D;
  // Conditional integration
  float u = actuator->sat(v);
  if((u - v) * e < 0.0) {
    I = I_prev;
  }
  if(ki == 0) {
    I = 0;
  }

  // Save states for the next iteration
  state[0] = I;
  state[1] = D;
  state[2] = e;
  state[3] = y;
  return u;
}

void PID::setParams(float values[]) {
  params[0] = values[0] / 12.0f; // kp
  params[1] = values[1] / 12.0f; // ki
  params[2] = values[2] / 12.0f; // kd
  params[3] = values[3] / 12.0f; // U0
  params[4] = values[4]; // N
  if(params[4] == 0) {
    params[4] = 20;
  }
  float Td = (params[0] > 0) ? params[2] / params[0] : 0;
  float N = params[4];
  float c = 2*Td + N*period;
  if(Td == 0) {
    params[5] = 0; // a 
    params[6] = 0; // b
  } else {
    params[5] = (2*Td - N*period) / c; // a 
    params[6] = - 2*params[2]*N / c; // b
  }
}
#endif

#ifdef USE_PICI
// -- PI+CI Controller --
PICI::PICI() {
  char* params_names[4] = {"kp", "ki", "pr", "u0"};
  char* state_names[3] = {"I", "Ic", "e_prev"};
  params = new float[4] {0.006, 0.002, 0.8, 0.93};
  state = new float[3] {0.0, 0.0, 0.0};
}

float PICI::update(float y) {
  float kp = params[0], ki = params[1], pr = params[2]; 
  float I = state[0], Ic =  state[1], e_prev = state[2];
  float e = sp - y;
  float P = kp*e;
  I += (ki*e) * (period/1000.0); // I
  Ic += (ki*e) * (period/1000.0); // Ic

  float v = u0 + P + (1-pr)*I + pr*Ic;
  // Conditional integration
  float u = actuator->sat(v);
  if((u - v) * e >= 0.0) {
    state[0] = I;
    state[1] = Ic;
  }
  // Reset
  if(e_prev * e <= 0) {
    state[1] = 0;
  }
  state[2] = e;
  return u;
}

void PICI::setParams(float values[]) {
  for(int i=0; i<4; i++) {
    params[i] = values[i];
  }
}
#endif

#ifdef USE_PI2D
// -- PI 2D Controller --
PI2D::PI2D() {
  char* params_names[6] = {"kp", "ki", "delta", "delta_I", "uff", "u0"};
  char* state_names[5] = {"I", "e_last", "I_last", "ysp", "uff"};
  params = new float[6] {0.006, 0.002, 1.0, 1.0, 0.1, 0.93};
  state = new float[5] {0.0, 0.0, 0.0, 0.0, 0.0};
}

float PI2D::update(float y) {
  float kp = params[0], ki = params[1], delta = params[2], delta_I = params[3], uff = params[4], u0 = params[5]; 
  float I = state[0], e_last = state[1], I_last = state[2], ysp = state[3], uff_last = state[4];
  float e = sp - y, I_prev = I;

  // feedforward
  if(sp != ysp) {
    uff_last = (sp > ysp) ? uff : - uff;
    ysp = sp;
  } else {
    if(abs(e) < delta) {
      uff_last = 0;
    }
  }

  // P Event
  if(abs(e - e_last) > delta) {
    e_last += (e > e_last) ? delta : -delta;
  }

  // I Event
  I += e * (period/1000.0f);
  if(abs(I - I_last) > delta_I) {
    I_last += (I > I_last) ? delta_I : -delta_I;
  }
  float v = u0 + uff_last + kp*e_last + ki*I_last;

/*  // Conditional integration */
  float u = actuator->sat(v);
  if((u - v) * e < 0.0) {
    I = I_prev;
  }

  state[0] = I;
  state[1] = e_last;
  state[2] = I_last;
  state[3] = ysp;
  state[4] = uff;

  return u;
}

void PI2D::setParams(float values[]) {
  for(int i=0; i<6; i++) {
    params[i] = values[i];
  }
}
#endif

#ifdef USE_PIARH
// -- PI ARH Controller --
PIARH::PIARH() {
  char* params_names[6] = {"kp", "ki", "rho", "alpha", "epsilon", "u0"};
  char* state_names[4] = {"I", "e_last", "xsi", "tau"};
  params = new float[6] {0.006, 0.002, 1.0, 0.5, 0.1, 0.9};
  state = new float[4] {0.0, 0.0, 0.0, 0.0};
}

float PIARH::update(float y) {
  float kp = params[0], ki = params[1], rho = params[2], alpha = params[3], epsilon = params[3], u0 = params[4];
  float I = state[0], e_prev = state[1], xsi = state[2], tau = state[3];
  float e = sp - y;
  float P = kp*e;
  float I_prev = I;

  tau += period/1000.0;
  I += (ki*e) * (period/1000.0);
  xsi += (ki*e) * (period/1000.0);
  float v = u0 + P + I;
  // Conditional integration
  float u = actuator->sat(v);
  if((u - v) * e < 0.0) {
    I = I_prev;
  }
  // Reset
  if(tau > rho && (xsi * (2*e + epsilon*xsi)) <= 0) {
    I = I - alpha*xsi;
    tau = 0;
    xsi = 0; 
  }
  
  state[0] = I;
  state[1] = e;
  state[2] = xsi;
  state[3] = tau;
  return u;
}

void PIARH::setParams(float values[]) {
  for(int i=0; i<5; i++) {
    params[i] = values[i];
  }
}
#endif
