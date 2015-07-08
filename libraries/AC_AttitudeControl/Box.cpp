#include "Box.h"
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"
#include "AP_Math.h"

extern const AP_HAL::HAL& hal;

#define PRINTERVAL 501

#define iprintf(...) if(printerval == 0) hal.console->printf(__VA_ARGS__)

/*
 * Distance traveled with initial velocity V,
 * acceleration a, for time t
 */
static float tdist(float V, float a, float t) {
    return (V*t) + ((a*t*t)/2.0f);
}

/*
 * Distance required to stop when traveling
 * at positive velocity V
 */
static float sdist(float V, float amin) {
    return -(V*V)/(2.0f*amin);
}

/*
 * Breaking acceleration in the Y direction
 */
float BoxShim::amin_Y() {
  return _params.amin;
}

/*
 * Breaking acceleration in the X direction
 */
float BoxShim::amin_X() {
  return (amin_Y() + gravity) * tan(_params.theta_min);
}

/*
 * The safety check for the position monitor.
 */
bool BoxShim::pos_safe_acc(float a, float v, float y,
			   float ub, float amin) {
  float mxa;
  if (a > 0.0f) {
    mxa = a;
  } else {
    mxa = 0.0f;
  }
  return y + tdist(v,mxa,d) + sdist(v + mxa*d,amin) <= ub;
}

/*
 * The safety check for the velocity monitor.
 */
bool BoxShim::vel_safe_acc(float a, float v, float ubv) {
  return a*d + v <= ubv;
}

/*
 * Returns polar coordinates such that A >= 0
 * -PI <= theta <= PI
 */
control_in BoxShim::rect_to_polar (float x, float y) {
  control_in res;
  res.theta = atan2(y,x);
  res.a = sqrt((x*x) + (y*y));
  return res;
}

/*
 * Returns the default action for one rectangular dimension.
 */
float BoxShim::default_rect_action_one_dim(float y, float v,
					   float amin) {
  if (y > 0 && v > 0) {
    return amin;
  } else if (y > 0 && v <= 0) {
    return 0;
  } else if (y <= 0 && v > 0) {
    return 0;
  } else {
    return -amin;
  }
}

/*
 * Returns the default action for the position
 * monitor in polar coordinates.
 *
 * The position monitor default action always passes
 * the velocity monitor's safety check, so we don't
 * have to consider it.
 */
control_in BoxShim::default_action(float x, float y, float vx, float vy) {
  float ax = default_rect_action_one_dim(x, vx,
					 amin_X());
  float ay = default_rect_action_one_dim(y, vy,
					 amin_Y());

  // We are rotated 90 degrees, so we reverse the arguments
  // We also account for gravity in the y direction
  return rect_to_polar(ay + gravity, ax);
}

static float shift(float ub, float lb) {
  return -((lb + ub)/2.0f);
}

static float bound_shift(float ub, float lb) {
  return (ub - lb)/2.0f;
}

/*
 * Runs the safety check in one dimension
 */
bool BoxShim::safe_acc(float a, float v, float y,
		       float ub, float ubv, float amin) {
  // ensure we don't print too often
  static int printerval = -1;
  printerval++;
  if (printerval == PRINTERVAL) printerval = 0;

  if (!vel_safe_acc(-a,-v,ubv)) {
    iprintf("Velneg: %f, %f, %f\n", -a, -v, ubv);
  }
  if (!pos_safe_acc(-a,-v,-y,ub,amin)) {
    iprintf("Posneg: %f, %f, %f, %f, %f\n", -a, -v, -y, ub, amin);
  }

  // iprintf("Velneg: %s\n", vel_safe_acc(-a,-v,ubv) ? "true" : "false");
  // iprintf("Velpos: %s\n", vel_safe_acc(a,v,ubv) ? "true" : "false");
  // iprintf("Pospos: %s\n", pos_safe_acc(a,v,y,ub,amin) ? "true" : "false");
  // iprintf("Posneg: %s\n", pos_safe_acc(-a,-v,-y,ub,amin) ? "true" : "false");

  // bool res =
  //   vel_safe_acc(-a,-v,ubv) && vel_safe_acc(a,v,ubv) &&
  //   pos_safe_acc(a,v,y,ub,amin) &&
  //   pos_safe_acc(-a,-v,-y,ub,amin);

  // iprintf("Result: %s\n", res ? "true" : "false");

  // return res;

  return
    vel_safe_acc(-a,-v,ubv) && vel_safe_acc(a,v,ubv) &&
    pos_safe_acc(a,v,y,ub,amin) &&
    pos_safe_acc(-a,-v,-y,ub,amin);
}

/*
 * Runs the monitor, returning a safe control input.
 */
control_in BoxShim::monitor(control_in proposed, state st) {

  // ensure we don't print too often
  static int printerval = -1;
  printerval++;
  if (printerval == PRINTERVAL) printerval = 0;

  float ubx = _params.ubx;
  float lbx = _params.lbx;
  float uby = _params.uby;
  float lby = _params.lby;
  float ubvx = _params.ubvx;
  float lbvx = _params.lbvx;
  float ubvy = _params.ubvy;
  float lbvy = _params.lbvy;

  float x = st.x+shift(ubx,lbx);
  float y = st.y+shift(uby,lby);
  float vx = st.vx+shift(ubvx,lbvx);
  float vy = st.vy+shift(ubvy,lbvy);
  float A = proposed.a;
  float Theta = proposed.theta;

  //iprintf("Theta_min: %s\n", _params.theta_min <= Theta ? "true" : "false");
  //iprintf("Theta_max: %s\n", Theta <= -_params.theta_min ? "true" : "false");
 
  // bool safe =
  //   safe_acc(A*sin(Theta), vx, x, bound_shift(ubx,lbx),
  // 	     bound_shift(ubvx,lbvx), amin_X()) &&
  //   safe_acc(A*cos(Theta)-gravity, vy, y, bound_shift(uby,lby),
  // 	     bound_shift(ubvy,lbvy), amin_Y()) &&
  //   _params.theta_min <= Theta &&
  //   Theta <= -_params.theta_min;
  bool safe1 =
    safe_acc(A*sin(Theta), vx, x, bound_shift(ubx,lbx),
	     bound_shift(ubvx,lbvx), amin_X());
  bool safe2 =
    safe_acc(A*cos(Theta)-gravity, vy, y, bound_shift(uby,lby),
	     bound_shift(ubvy,lbvy), amin_Y());

  //  iprintf("Safe1: %s\n", safe1 ? "true" : "false");
  //  iprintf("Safe2: %s\n", safe2 ? "true" : "false");
  //iprintf("Bound: %f\n", bound_shift(uby,lby));

  // Issue the proposed signal if it passes
  // all safety checks, otherwise issue the default
  if (safe_acc(A*sin(Theta), vx, x, bound_shift(ubx,lbx),
	       bound_shift(ubvx,lbvx), amin_X()) &&
      safe_acc(A*cos(Theta)-gravity, vy, y, bound_shift(uby,lby),
	       bound_shift(ubvy,lbvy), amin_Y()) &&
      _params.theta_min <= Theta &&
      Theta <= -_params.theta_min) {
    iprintf("Issuing proposed\n");
    control_in res = proposed;
    res.updated = false;
    return res;
  } else {
    iprintf("Issuing default\n");
    control_in res = default_action(x, y, vx, vy);
    res.updated = true;
    return res;
  }
}

float BoxShim::get_x() {
  return _inav.get_position().y;
}

float BoxShim::get_y() {
  return _inav.get_position().z;
}

float BoxShim::get_vx() {
  return _inav.get_velocity().y;
}

float BoxShim::get_vy() {
  return _inav.get_velocity().z;
}

float BoxShim::throttle_to_accel_scale() {
  return gravity/500.0f;
}

// Gives an estimate of an upper bound on acceleration
// using the throttle value.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float BoxShim::get_acc_from_throttle(float throttle) {
  return throttle*throttle_to_accel_scale();
}

float BoxShim::get_throttle_from_acc(float A) {
  return A/throttle_to_accel_scale();
}

void BoxShim::attitude_shim_entry_point(Att_shim_params params, bool first_call) {

  _params = {-500.0f, 1000.0f, -1000.0f, 6000.0f, -2000.0f, 500.0f, -500.0f, 500.0f, -500.0f, -(M_PI + 0.0f)/4.0f };

  static float roll = 0.0f;
  static int16_t throttle = 0;

  switch (params.which_fn) {

  case THROTTLE_SET:

    throttle = params.throttle;

    break;
    
  default:

    roll = params.roll;

  }

  // ensure we don't print too often
  static int printerval = -1;
  printerval++;
  if (printerval == PRINTERVAL) printerval = 0;

  iprintf("Theta_min: %f\n", _params.theta_min);

  control_in proposed;
  proposed.theta = radians(wrap_180_cd_float(roll)/100.0f);
  proposed.a = get_acc_from_throttle(throttle + 0.0f);

  state st;
  st.x = get_x();
  st.y = get_y();
  st.vx = get_vx();
  st.vy = get_vy();

  iprintf("Y: %f, VY: %f, X: %f, VX: %f\n", st.y, st.vy, st.x, st.vx);

  iprintf("About to check safety of A: %f, Theta: %f\n", proposed.a, proposed.theta);

  control_in safe = monitor(proposed, st);

  if (safe.updated) {
    params.roll = degrees(safe.theta);
    params.throttle = get_throttle_from_acc(safe.a);
    params.angle_boost = false;
  }

  iprintf("Safe roll: %f\n", params.roll);
  iprintf("Safe throttle: %u\n", params.throttle);

  AC_AttitudeShim::attitude_shim_entry_point(params, first_call);

}
