#pragma once
#include "control.h"

// A classic PID Controller with antiwindup mechanism
class OPT2 : public Controller {
	private:
		float period = 100.0;
		float u0 = 0.9;
	public:
		OPT2();
		float update(float y);
		void setKp(float kp);
		void setKi(float ki);
		void setKd(float kd);
		void setParams(float value[]);
};


