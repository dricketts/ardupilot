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

struct polar {
  float a;
  float theta;
};

struct control_in {
  float a;
  float pitch;
  float roll;
  bool updated;
};

struct state {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
};

struct bounds {
  float ubx;
  float uby;
  float ubz;
  float ubvx;
  float ubvy;
  float ubvz;
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
  safety_check safe_z;
  bool Pitch_bound_check;
  bool Roll_bound_check;
  float ax;
  float ay;
  float az;
  control_in cin;
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
  void set_z_ub(float b) {_z_ub = b;}
  void set_z_lb(float b) {_z_lb = b;}
  void set_vz_ub(float b) {_vz_ub = b;}
  void set_vz_lb(float b) {_vz_lb = b;}
  void set_angle_lb(float r) {_angle_lb = r;}
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
  float z_ub() {return _z_ub;}
  float z_lb() {return _z_lb;}
  float vz_ub() {return _vz_ub;}
  float vz_lb() {return _vz_lb;}
  float angle_lb() {return _angle_lb;}
  float abraking() {return _abraking;}
  float lookahead() {return _lookahead;}

  /*
   * Returns polar coordinates such that A >= 0
   * -PI <= theta <= PI
   */
  static polar rect_to_polar (float x, float y);

  /*
   * Returns spherical coordinates such that A >= 0
   * -PI <= pitch <= PI and -PI/2 <= roll <= PI/2
   */
  static control_in rect_to_spherical (float x, float y, float z);

 private:

  /*
   * Functions for computing if this monitor can run.
   */
  bool ind_inv_acc(float y, float v, float ub, float amin);
  bool ind_inv_vel(float v, float ub);
  bool ind_inv(float vx, float vy, float vz, float x, float y, float z, float ubx,
	       float ubvx, float uby, float ubvy, float ubz, float ubvz);

  /*
   * Implements the actual monitor logic.
   */
  monitor_check monitor_logic(float AX, float AY, float AZ, float Pitch, float Roll,
			      float vx, float vy, float vz, float x, float y, float z,
			      float ubx, float ubvx, float uby, float ubvy,
			      float ubz, float ubvz);

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

  /*
   * Breaking acceleration in the Z direction
   */
  float amin_Z();

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
  // horizontal bounds
  float _z_ub;
  float _z_lb;
  // horizontal velocity bounds
  float _vz_ub;
  float _vz_lb;
  // roll and pitch bounds
  float _angle_lb;
  // braking acceleration for vertical dimension
  float _abraking;
  float _lookahead;
};

#endif //Box_H
