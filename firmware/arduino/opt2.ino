// -- PID Controller --
OPT2::OPT2() {
  char* params_names[4] = {"t1", "u1", "u2", "d"};
  char* state_names[3] = {"t", "u", "next"};
  params = new float[4] {0.0, 0.0, 0.9, 0.0};
  state = new float[3] {0.0, 0.0,0.0};
}

float OPT2::update(float y) {
  noInterrupts();
  float t1 = params[0], u1 = params[1], u2 = params[2], d = params[3];
  float t = state[0], u = state[1], next = state[2];
  interrupts();

  t += 0.02f;
  
  if(next == 1) { //&& t >= d
    t = 0;
    u = u1;
    next = 2;
  } else if(next == 2 && t > t1) {
    t = 0;
    u = u2;
    next = 0;
  }

  noInterrupts();
  state[0] = t;
  state[1] = u;
  state[2] = next;
  interrupts();
  return u;
}

void OPT2::setParams(float values[]) {
  noInterrupts();
  for(int i=0; i<4; i++) {
    params[i] = values[i];
  }
  state[2] = 1;
  interrupts();
}


