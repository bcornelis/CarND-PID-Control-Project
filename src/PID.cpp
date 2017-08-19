#include "PID.h"
#include <iomanip>
#include <math.h>

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  //Init(1.0011, 0.25339, 3.22);
  //Init(0.35, 0.01, 0.004);
  Init(0.13, 0.0, 3.3);
  //twiddle_enabled = true;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = this->i_error = this->d_error = 0.;

  twiddle_index = 0;
  twiddle_num_steps_for_evaluation = 25;
  twiddle_enabled = false;
  twiddle_best_error = 999999999999.9;
  dp[0] = .001;
  dp[1] = .1;
  dp[2] = .001;
  twiddle_tolerance = 0.001;
  twiddle_first_stage = true;
}

void PID::logTwiddleStatus(string extraMessage) {
  std::cout << "[twiddle] index=" << twiddle_index << std::setprecision(5);
  std::cout << "; dp[0]="<< dp[0] << "; dp[1]=" << dp[1] << "; dp[2]="<<dp[2];
  std::cout << "; Ki="<< Ki << "; Kp=" << Kp << "; Kd=" << Kd;
  std::cout << "; error=" << twiddle_error;
  std::cout << " **** " << extraMessage << std::endl;
}

void PID::UpdateError(double cte) {
  // the d-error is the current CTE minus the previous one
  d_error = cte - p_error;
  // the p-error - simply the CTE
  p_error = cte;
  // the i-error is the total error
  i_error += cte;

  // TODO: even if twiddling has ended, this will overflow
  if( twiddle_enabled ) {
    twiddle_error += cte;
    twiddle_step+=1;
  }

  // need to handle twiddle step?
  if( twiddle_enabled && (twiddle_step % twiddle_num_steps_for_evaluation) == 0) {

    // always disable twiddling if the error is small enough
    if( dp[0] + dp[1] + dp[2] <= twiddle_tolerance) {
      // stop twiddling!
      twiddle_enabled = false;
      logTwiddleStatus("Done!");
      // and stop doing anything
      return;
    } else {
      logTwiddleStatus("continuing...");
    }

    twiddle_error = fabs(twiddle_error);

    // if we are in the first stage
    if(twiddle_first_stage) {

      // always update
      twiddle_update_p(1.);

      if( twiddle_error < twiddle_best_error) {
        twiddle_best_error = twiddle_error;
        dp[twiddle_index] *= 1.1;
        rollTwiddleIndex();
      } else {
        twiddle_update_p(-2.);
        twiddle_first_stage = false;
      }
    } else {
      // second stage run
      if( twiddle_error < twiddle_best_error) {
        twiddle_best_error = twiddle_error;
        dp[twiddle_index] *= 1.1;
      } else {
        twiddle_update_p(1.);
        dp[twiddle_index] *= .9;
      }

      rollTwiddleIndex();
      twiddle_first_stage=true;
    }

    // cleanup for the next one
    twiddle_error = 0;
  }
}

void PID::twiddle_update_p(double correctionFactor) {
  switch(twiddle_index) {
  case 0:
    Kp += correctionFactor * dp[twiddle_index];
    break;
  case 1:
    Ki += correctionFactor * dp[twiddle_index];
    break;
  case 2:
    Kd += correctionFactor * dp[twiddle_index];
  }
}

void PID::rollTwiddleIndex() {
  twiddle_index = 1 + ((twiddle_index+1) %2);
}

double PID::TotalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

