#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

/**
 * Initialize PID.
 * @param kp the proportional value for PID controller
 * @param ki the integral value for PID controller
 * @param kd the derivative value for PID controller
 */
void PID::Init(double kp, double ki, double kd) {
  	Kp = kp;
  	Ki = ki;
  	Kd = kd;

  	p_error = 0;
  	i_error = 0;
	d_error = 0;
}


/**
 * Update the PID error variables given cross track error
 * @param cte the current cross track error
 */
void PID::UpdateError(double cte) {

  	double previous_cte = p_error;

  	// Proportional error is just the CTE (Cross Track Error)
  	p_error = cte;

  	// Integration error is the sum of all CTE so far
  	i_error = i_error + cte;

  	// Differential error is the rate of change of the CTE but assuming
  	// each call is 1 time-step then it simplifies to subtracting the 
  	// previous CTE from the current CTE.
	d_error = cte - previous_cte;
}


double PID::TotalError() {
	// Multiply out our errors by the coefficients
	return -Kp * p_error - Ki * i_error - Kd * d_error;
}


