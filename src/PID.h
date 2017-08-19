#ifndef PID_H
#define PID_H

#include <iostream>

using namespace std;


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  // all twiddling related
  bool twiddle_enabled;
  bool twiddle_first_stage;
  int twiddle_index;
  int twiddle_step;
  int twiddle_num_steps_for_evaluation;
  double twiddle_error;
  double twiddle_best_error;
  double twiddle_tolerance;
  double dp[3];

  void logTwiddleStatus(string extraMessage);
  void twiddle_update_p(double correctionFactor);
  void rollTwiddleIndex();
};

#endif /* PID_H */
