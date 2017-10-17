#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  // PID error terms
  double p_error_; // proportional error term
  double i_error_; // integral error term
  double d_error_; // derivative error term
  
  // Stored values for filtering
  double prev_cte_; // previous cross-track error for D term latching
  double prev_d_error_; // previous D error term for smoothing
  double prev_total_error_; // previous total error for output rate limiting
  
  // PID tweaking parameters
  double i_max_; // I term max guard value
  bool i_cut_; // I term manual cut flag
  double d_max_; // D term max guard value
  double d_smooth_; // D term smoothing factor (1 = no smoothing)
  double error_rate_max_; // total error max rate limit value
  
  // PID gain coefficients
  double Kp_; // P gain
  double Ki_; // I gain
  double Kd_; // D gain
  
  // Twiddle parameters
  std::vector<double*> Kgains_; // array of pointers to PID gains to be twiddled
  std::vector<double> Kdeltas_; // array of PID deltas to be twiddled
  double twiddle_error_; // twiddle error function value
  double twiddle_best_error_; // stored best error value
  int twiddle_idx_; // PID gain index (0 = P, 1 = I, 2 = D)
  bool twiddle_switch_; // switch for twiddle direction (true = decrease gain,
                        //   false = increase gain)

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor
   */
  virtual ~PID();

  /**
   * Reset PID terms to initial values
   */
  void Reset();
  
  /**
   * Initialize PID object
   */
  void Init(double Kp, double Ki, double Kd,
            double i_max = __DBL_MAX__, double d_max = __DBL_MAX__,
            double d_smooth = 1.0, double error_rate_max = __DBL_MAX__);

  /**
   * Update the PID error terms given Cross Track Error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error output
   */
  double TotalError();
  
  /**
   * Limiting functions
   */
  double RateLimit(double raw_value, double prev_value, double rate_max);
  double MinMaxLimit(double raw_value, double minmax_limit);
  
  /**
   * Twiddle algorithm functions
   */
  void TwiddleErrorUpdate(double cte, double steer);
  void TwiddleParamUpdate();
};

#endif /* PID_H */
