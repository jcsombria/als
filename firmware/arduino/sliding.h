#pragma once
#include "control.h"

class Sliding : public Controller {
	private:
		float period = 20.0;
		float u0 = 0.9;
	public:
		Sliding();
		float update(float y);
		void setParams(float value[]);
};


