#include "Box.h"
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"
#include "AP_Math.h"
#include <time.h>

extern const AP_HAL::HAL& hal;

#define PRINTERVAL 1000

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
  return abraking();
}

/*
 * Breaking acceleration in the X direction
 */
float BoxShim::amin_X() {
  return (amin_Y() + gravity) * tan(roll_lb());
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
  return y + tdist(v,mxa,_d_ctrl) + sdist(v + mxa*_d_ctrl,amin) <= ub;
}

/*
 * The safety check for the velocity monitor.
 */
bool BoxShim::vel_safe_acc(float a, float v, float ubv) {
  return a*_d_ctrl + v <= ubv;
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

static bool all_safe(safety_check c) {
  return c.vel_lb && c.vel_ub && c.pos_ub && c.pos_lb;
}

/*
 * Runs the safety check in one dimension
 */
safety_check BoxShim::safe_acc(float a, float v, float y,
		       float ub, float ubv, float amin) {

  safety_check res;
  res.vel_lb = vel_safe_acc(-a,-v,ubv);
  res.vel_ub = vel_safe_acc(a,v,ubv);
  res.pos_ub = pos_safe_acc(a,v,y,ub,amin);
  res.pos_lb = pos_safe_acc(-a,-v,-y,ub,amin);
  return res;
}

/*
 * Runs the monitor, returning a safe control input.
 */
control_in BoxShim::monitor(control_in proposed, state st) {

  float ubx = x_ub();
  float lbx = x_lb();
  float uby = h_ub();
  float lby = h_lb();
  float ubvx = xprime_ub();
  float lbvx = xprime_lb();
  float ubvy = hprime_ub();
  float lbvy = hprime_lb();

  float x = st.x+shift(ubx,lbx);
  float y = st.y+shift(uby,lby);
  float vx = st.vx+shift(ubvx,lbvx);
  float vy = st.vy+shift(ubvy,lbvy);
  float A = proposed.a;
  float Theta = proposed.theta;
  float AX = A*sin(Theta);
  float AY = A*cos(Theta)-gravity;
  float ax;
  float ay;

  safety_check safe_check_x = safe_acc(AX, vx, x, bound_shift(ubx,lbx),
				       bound_shift(ubvx,lbvx), amin_X());
  bool safe_x = all_safe(safe_check_x);
  safety_check safe_check_y = safe_acc(AY, vy, y, bound_shift(uby,lby),
				       bound_shift(ubvy,lbvy), amin_Y());
  bool safe_y = all_safe(safe_check_y);
  bool Theta_bound_check = roll_lb() <= Theta && Theta <= -roll_lb();

  control_in res;

  // Issue the proposed signal if it passes
  // all safety checks, otherwise issue the default
  if (safe_x && safe_y && Theta_bound_check) {
    ax = AX;
    ay = AY;

    res = proposed;
    res.updated = false;
  } else if (safe_x && amin_X() <= AX && AX <= -amin_X()) {
    ax = AX;
    ay = default_rect_action_one_dim(y, vy, amin_Y());

    res = rect_to_polar(ay + gravity, AX);
    res.updated = true;
  } else if (safe_y && amin_Y() <= AY && AY <= -amin_Y()) {
    ax = default_rect_action_one_dim(x, vx, amin_X());
    ay = AY;

    res = rect_to_polar(AY + gravity, ax);
    res.updated = true;
  } else {

    ax = default_rect_action_one_dim(x, vx, amin_X());
    ay = default_rect_action_one_dim(y, vy, amin_Y());

    // We are rotated 90 degrees, so we reverse the arguments
    // We also account for gravity in the y direction
    res = rect_to_polar(ay + gravity, ax);

    //    control_in res = default_action(x, y, vx, vy);
    res.updated = true;
  }

  // Set all the statistics
  _stats.x = x;
  _stats.y = y;
  _stats.vx = vx;
  _stats.vy = vy;
  _stats.A = A;
  _stats.Theta = Theta;
  _stats.a = res.a;
  _stats.theta = res.theta;
  _stats.AX = AX;
  _stats.AY = AY;
  _stats.ax = ax;
  _stats.ay = ay;
  _stats.amin_x = amin_X();
  _stats.amin_y = amin_Y();
  _stats.safe_x = safe_x;
  _stats.safe_y = safe_y;
  _stats.safe_x_vel_ub = safe_check_x.vel_ub;
  _stats.safe_x_vel_lb = safe_check_x.vel_lb;
  _stats.safe_x_pos_ub = safe_check_x.pos_ub;
  _stats.safe_x_pos_lb = safe_check_x.pos_lb;
  _stats.safe_y_vel_ub = safe_check_y.vel_ub;
  _stats.safe_y_vel_lb = safe_check_y.vel_lb;
  _stats.safe_y_pos_ub = safe_check_y.pos_ub;
  _stats.safe_y_pos_lb = safe_check_y.pos_ub;
  _stats.Theta_bound_check = Theta_bound_check;

  return res;
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
  return gravity/mid_throttle();
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

  //_params = {-500.0f, 10000.0f, -10000.0f, 6000.0f, -2000.0f, 500.0f, -500.0f, 500.0f, -500.0f, -(M_PI + 0.0f)/4.0f };

  if (shim_on()) {
    static float roll = 0.0f;
    static int16_t throttle = 0;

    switch (params.which_fn) {

    case THROTTLE_SET:

      throttle = params.throttle;

      break;
    
    default:

      roll = params.roll;

    }

    control_in proposed;
    proposed.theta = radians(wrap_180_cd_float(roll)/100.0f);
    proposed.a =
      get_acc_from_throttle(params.angle_boost ?
			    get_angle_boost(throttle + 0.0f) :
			    throttle + 0.0f);

    state st;
    st.x = get_x();
    st.y = get_y();
    st.vx = get_vx();
    st.vy = get_vy();

    control_in safe = monitor(proposed, st);

    if (safe.updated) {
      params.roll = degrees(safe.theta)*100.0f;
      params.throttle = get_throttle_from_acc(safe.a);
      params.angle_boost = false;
    }

  }

  AC_AttitudeShim::attitude_shim_entry_point(params, first_call);

}
