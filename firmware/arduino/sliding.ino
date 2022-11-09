Sliding::Sliding() {
  char* params_names[4] = {"k1", "k2", "k3","u0"};
  char* state_names[4] = {"e_prev", "de_prev","u"};
  state = new float[4] {0.0, 0.0, 0.0};
  params = new float[4] {1.0, 0.2, 0.003, 0.6};
}

float Sliding::update(float y) {
  float k1 = params[0], k2 = params[1], k3 = params[2], u0 = params[3];
  float e_prev = state[0], de_prev = state[1], uf = state[2];

  float e = y - sp;
  float de = (e - e_prev) / (period / 1000.0);
  float dde = (de - de_prev) / (period / 1000.0);

  float s = k1*e + de, sign = (s > 0) ? 1.0 : -1.0;
//  float s = k1*k1*e + 2*k1*de + dde, sign = (s > 0) ? 1.0 : -1.0;
  float u = u0 + k3*sp - sign*k2;
  
  float u_sat = actuator->sat(u);

  state[0] = e;
  state[1] = de;
  state[2] = u;
  return u_sat;
}

void Sliding::setParams(float values[]) {
  for(int i=0; i<4; i++) {
    params[i] = values[i];
  }
}


