#ifndef Box_H
#define Box_H

#include <math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"

// Acceleration due to gravity in cm/s/s.
// Sensors seem to use cm as length unit.
// This constant has same level of precision
// as python simulation.
static const float gravity = 980.665f;

struct control_in {
  float a;
  float theta;
  bool updated;
};

struct state {
  float x;
  float y;
  float vx;
  float vy;
};

struct safety_check {
  bool vel_ub;
  bool vel_lb;
  bool pos_ub;
  bool pos_lb;
};

struct monitor_check {
  safety_check safe_x;
  safety_check safe_y;
  bool Theta_bound_check;
  float ax;
  float ay;
  control_in cin;
};

class BoxShim : public AC_AttitudeShim {
 public:
  BoxShim(AP_AHRS &ahrs,
	  const AP_Vehicle::MultiCopter &aparm,
	  AP_Motors& motors,
	  AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
	  AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
    const float h_ub, const float h_lb, const float hprime_ub, const float hprime_lb,
    const float x_ub, const float x_lb, const float xprime_ub, const float xprime_lb,
    const float roll_lb, const float abraking, const float mid_throttle,
    const float d_ctrl,
	  const AP_InertialNav& inav
	  ) :
  AC_AttitudeShim(ahrs, aparm, motors, pi_angle_roll, pi_angle_pitch,
		  pi_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw,
      h_ub, h_lb, hprime_ub, hprime_lb, x_ub, x_lb, xprime_ub, xprime_lb,
		  roll_lb, abraking, mid_throttle, d_ctrl),
    _inav(inav)
    {
    };

  float get_x();
  float get_y();
  float get_vx();
  float get_vy();

 private:

  void attitude_shim_entry_point(Att_shim_params params, bool first_call);

  float throttle_to_accel_scale();

  float get_acc_from_throttle(float throttle);
  float get_throttle_from_acc(float A);


  /*
   * Runs the monitor, returning a safe control input.
   */
  control_in monitor(control_in proposed, state st);

  /*
   * Implements the actual monitor logic.
   */
  monitor_check monitor_logic(float AX, float AY, float Theta, float vx,
			      float vy, float x, float y, float ubx,
			      float ubvx, float uby, float ubvy);

  /*
   * Computes the maximum allowed acceleration by the velocity shim
   */
  float max_acc_velocity(float v, float ub);

  /*
   * Computes the maximum allowed acceleration by the position shim
   */
  float max_acc_position(float y, float v, float ub, float amin);

  /*
   * Used by max_acc_position and min_acc_position
   */
  float sqrt_expr(float y, float v, float ub, float amin);

  /*
   * Runs the safety check in one dimension
   */
  safety_check safe_acc(float a, float v, float y,
			float ub, float ubv, float amin);

  /*
   * Returns the default action for the position
   * monitor in polar coordinates.
   *
   * The position monitor default action always passes
   * the velocity monitor's safety check, so we don't
   * have to consider it.
   */
  control_in default_action(float x, float y, float vx, float vy);

  /*
   * Returns the default action for one rectangular dimension.
   */
  float default_rect_action_one_dim(float y, float v,
				    float amin);

  /*
   * Returns polar coordinates such that A >= 0
   * -PI <= theta <= PI
   */
  control_in rect_to_polar (float x, float y);

  /*
   * The safety check for the velocity monitor.
   */
  bool vel_safe_acc(float a, float v, float ubv);

  /*
   * The safety check for the position monitor.
   */
  bool pos_safe_acc(float a, float v, float y, float ub,
		    float amin);

  /*
   * Breaking acceleration in the X direction
   */
  float amin_X();

  /*
   * Breaking acceleration in the Y direction
   */
  float amin_Y();

  const AP_InertialNav& _inav;
};

#endif //Box_H
