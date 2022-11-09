#pragma once
#include "control.h"

#ifdef USE_PID
// A classic PID Controller with antiwindup mechanism
class PID : public Controller {
	private:
		float period = 100.0f / 1000.0f;
		float u0 = 0.6;
	public:
		PID();
		float update(float y);
		void setKp(float kp);
		void setKi(float ki);
		void setKd(float kd);
		void setParams(float value[]);
};
#endif

#ifdef USE_PICI
// A PI controller with a Clegg's integrator (\cite{Baños2009})
class PICI : public Controller {
	private:
		float period = 100.0;
		float u0 = 0.9;
	public:
		PICI();
		float update(float y);
		void setParams(float value[]);
};
#endif

#ifdef USE_PI2D
// An event-based PI controller with feedforward (\cite{Sanchez2011})
class PI2D : public Controller {
	private:
		float period = 100.0;
		float u0 = 0.9;
	public:
		PI2D();
		float update(float y);
		void setParams(float value[]);
};
#endif

#ifdef USE_PIARH
// An adaptive robust hybrid PI controller (\cite{Scola2017})
class PIARH : public Controller {
	private:
		float period = 100.0;
		float u0 = 0.9;
	public:
		PIARH();
		float update(float y);
		void setParams(float value[]);
};
#endif


