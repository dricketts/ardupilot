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

struct bounds {
  float ubx;
  float uby;
  float ubvx;
  float ubvy;
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

// statistics about the shim
struct box_shim_stats {
  float x;
  float y;
  float vx;
  float vy;
  uint16_t throttle;
  bool angle_boost;
  float A;
  float Theta;
  float a;
  float theta;
  float AX;
  float AY;
  float AX_check;
  float AY_check;
  float ax;
  float ay;
  float amin_x;
  float amin_y;
  bool safe_x;
  bool safe_y;
  bool safe_x_vel_ub;
  bool safe_x_vel_lb;
  bool safe_x_pos_ub;
  bool safe_x_pos_lb;
  bool safe_y_vel_ub;
  bool safe_y_vel_lb;
  bool safe_y_pos_ub;
  bool safe_y_pos_lb;
  bool Theta_bound_check;
};

class BoxShim {
 public:
  BoxShim() {};

  /*
   * Returns true iff the monitor can be run.
   */
  bool can_run(state st);

  /*
   * Runs the monitor, returning a safe control input.
   */
  control_in monitor(control_in proposed, state st);

  // set shim bounding parameters
  void set_y_ub(float b) {_y_ub = b;}
  void set_y_lb(float b) {_y_lb = b;}
  void set_vy_ub(float b) {_vy_ub = b;}
  void set_vy_lb(float b) {_vy_lb = b;}
  void set_x_ub(float b) {_x_ub = b;}
  void set_x_lb(float b) {_x_lb = b;}
  void set_vx_ub(float b) {_vx_ub = b;}
  void set_vx_lb(float b) {_vx_lb = b;}
  void set_roll_lb(float r) {_roll_lb = r;}
  void set_abraking(float a) {_abraking = a;}
  void set_lookahead(float x) {_lookahead = x;}
  void set_d_ctrl(float d_ctrl) {_d_ctrl = d_ctrl;}
  void set_smooth(bool smooth) {_smooth = smooth;}

  float d_ctrl() {return _d_ctrl;}
  bool smooth() {return _smooth;}
  float y_ub() {return _y_ub;}
  float y_lb() {return _y_lb;}
  float vy_ub() {return _vy_ub;}
  float vy_lb() {return _vy_lb;}
  float x_ub() {return _x_ub;}
  float x_lb() {return _x_lb;}
  float vx_ub() {return _vx_ub;}
  float vx_lb() {return _vx_lb;}
  float roll_lb() {return _roll_lb;}
  float abraking() {return _abraking;}
  float lookahead() {return _lookahead;}

  //
  // Stats Reporing (Accessor)
  //
  box_shim_stats get_shim_stats() {return _stats;}

  /*
   * Returns polar coordinates such that A >= 0
   * -PI <= theta <= PI
   */
  static control_in rect_to_polar (float x, float y);

 private:

  void attitude_shim_entry_point(Att_shim_params params, bool first_call);

  float throttle_to_accel_scale();

  float get_acc_from_throttle(float throttle);
  float get_throttle_from_acc(float A);

  /*
   * Functions for computing if this monitor can run.
   */
  bool ind_inv_acc(float y, float v, float ub, float amin);
  bool ind_inv_vel(float v, float ub);
  bool ind_inv(float vx, float vy, float x, float y, float ubx,
	       float ubvx, float uby, float ubvy);

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
  float sqrt_expr(float y, float v, float ub, float amin, float d);

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

  state shift_state(state st);
  bounds shift_bounds();

  bool _smooth;
  // delay associated with actuators
  float _d_ctrl;
  // height bounds
  float _y_ub;
  float _y_lb;
  // vertical velocity bounds
  float _vy_ub;
  float _vy_lb;
  // horizontal bounds
  float _x_ub;
  float _x_lb;
  // horizontal velocity bounds
  float _vx_ub;
  float _vx_lb;
  // roll bounds
  float _roll_lb;
  // braking acceleration for vertical dimension
  float _abraking;
  float _lookahead;
  box_shim_stats _stats;
};

#endif //Box_H
