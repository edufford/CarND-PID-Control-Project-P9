#include "PID.h"
#include <cmath>
#include <iostream>

PID::PID() {}

PID::~PID() {}

/**
 * PID reset to clear error terms for each drive.
 */
void PID::Reset() {
  // Initialize error terms
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  
  // Initialize stored previous values
  prev_d_error_ = 0.0;
  prev_total_error_ = 0.0;
  
  // Initialize twiddle error term
  twiddle_error_ = 0.0;
}

/**
 * PID initialization to set gains and other stored parameters.  Reset
 * function is also called to clear error terms.
 */
void PID::Init(double Kp, double Ki, double Kd, double i_max, double d_max,
               double d_smooth, double error_rate_max) {
  // Initialize parameters
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  i_max_ = i_max;
  i_cut_ = false;
  d_max_ = d_max;
  d_smooth_ = d_smooth;
  error_rate_max_ = error_rate_max;

  // Initialize twiddle parameters
  Kgains_ = {&Kp_, &Ki_, &Kd_};
  Kdeltas_ = {0.05, 0.0005, 1.0};
  twiddle_best_error_ = __DBL_MAX__;
  twiddle_idx_ = 0;
  twiddle_switch_ = false;

  // Reset PID terms to initial values
  Reset();
}

/**
 * PID function to calculate each error term (Proportional, Integral, and
 * Derivative).  I term includes min/max guard for integral windup and manual
 * cut flag for standing start condition.  D term includes latching until next
 * discrete cte update, smoothing, and min/max guard to prevent spikes.
 */
void PID::UpdateError(double cte) {
  // P term
  p_error_ = -Kp_ * cte;
  
  // I term with max windup limit and manual cut
  if (i_cut_ == false) {
    i_error_ += -Ki_ * cte;
    i_error_ = MinMaxLimit(i_error_, i_max_);
  }
  else {
    i_error_ = 0.0;
  }
  
  // D term latched until next cte update
  if (cte != prev_cte_) {
    d_error_ = -Kd_ * (cte - prev_cte_);

    // Smoothing
    d_error_ = prev_d_error_ * (d_smooth_-1.0)/d_smooth_ + d_error_/d_smooth_;
    
    // Max limit
    d_error_ = MinMaxLimit(d_error_, d_max_);
    
    prev_d_error_ = d_error_;
  }

  prev_cte_ = cte;
}

/**
 * PID function to calculate total error output (P + I + D terms), with rate
 * limit filtering and min/max guard to maintain smoother steering control.
 *
 * Returns the final total error feedback value (steering control value).
 */
double PID::TotalError() {
  // Calculate PID feedback amount
  double total_error_raw = p_error_ + i_error_ + d_error_;

  // Rate limit to error_rate_max
  double total_error_filt = RateLimit(total_error_raw, prev_total_error_,
                                       error_rate_max_);
  
  // Clip total error between [-1.0, 1.0] for steering control value
  total_error_filt = MinMaxLimit(total_error_filt, 1.0);
  
  // Store as previous value for next loop's filtering
  prev_total_error_ = total_error_filt;
  
  return total_error_filt;
}

/**
 * Max rate limit function
 */
double PID::RateLimit(double raw_value, double prev_value, double rate_max) {
  double rate_limited_value = raw_value;
  
  if ((raw_value - prev_value) > rate_max) {
    rate_limited_value = prev_value + rate_max;
  }
  else if ((raw_value - prev_value) < -rate_max) {
    rate_limited_value = prev_value - rate_max;
  }
  
  return rate_limited_value;
}

/**
 * Min/Max guard function
 */
double PID::MinMaxLimit(double raw_value, double minmax_limit) {
  double limited_value = raw_value;
  
  if (limited_value > minmax_limit) {
    limited_value = minmax_limit;
  }
  else if (limited_value < -minmax_limit) {
    limited_value = -minmax_limit;
  }
  
  return limited_value;
}

/**
 * Twiddle error term set as accumulated CTE and steering angle to optimize for
 * smaller cross track error but also smaller and smoother steering work.
 */
void PID::TwiddleErrorUpdate(double cte, double steer) {
  twiddle_error_ += abs(steer);
  twiddle_error_ += abs(cte);
}

/**
 * Twiddle algorithm to decide next parameter set based on the twiddle error
 * result.  Twiddle steps are:
 *   1.  Increase gain by its delta and do a run to check new error.
 *   2a. If error improved, boost the delta and move to the next gain index.
 *   2b. If error was worse, decrease gain by the delta in the other direction.
 *   3a. If error improved in the other direction, boost the delta and move to
 *        the next gain index.
 *   3b. If error was worse in the other direction, set the gain back to the
 *        original value and reduce the delta, then move to the next gain index.
 */
void PID::TwiddleParamUpdate() {
  if (twiddle_switch_ == false) { // (test gain in the increased direction)
    if (twiddle_error_ < twiddle_best_error_) {
      twiddle_best_error_ = twiddle_error_;
      Kdeltas_[twiddle_idx_] *= 1.2;
      twiddle_idx_ = (twiddle_idx_ + 1) % Kgains_.size();
      *Kgains_[twiddle_idx_] += Kdeltas_[twiddle_idx_];
      // Found new best error, boost delta and move to next gain index
      std::cout << "Error better, boost delta and move to next index."
                << std::endl;
    }
    else {
      *Kgains_[twiddle_idx_] -= 2 * Kdeltas_[twiddle_idx_];
      twiddle_switch_ = true;
      // Switch direction and try again
      std::cout << "Error worse, try other direction." << std::endl;
    }
  }
  else { // twiddle_switch_ = true (retest gain in the decreased direction)
    if (twiddle_error_ < twiddle_best_error_) {
      twiddle_best_error_ = twiddle_error_;
      Kdeltas_[twiddle_idx_] *= 1.2;
      // Found new best error, boost delta
      std::cout << "Error better in this direction, boost delta." << std::endl;
    }
    else {
      // Neither direction improved error, set gain back to original and reduce
      // size of delta
      *Kgains_[twiddle_idx_] += Kdeltas_[twiddle_idx_];
      Kdeltas_[twiddle_idx_] *= 0.8;
      std::cout << "Error still worse in this direction,"
                << "set back gain and reduce delta." << std::endl;
    }
    // Move to next gain index
    std::cout << "Twiddle next gain index." << std::endl;
    twiddle_idx_ = (twiddle_idx_ + 1) % Kgains_.size();
    *Kgains_[twiddle_idx_] += Kdeltas_[twiddle_idx_];
    twiddle_switch_ = false;
  }
}
