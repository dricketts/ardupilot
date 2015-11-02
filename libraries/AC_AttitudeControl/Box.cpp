#include "Box.h"
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"
#include "AP_Math.h"

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
 * Used by max_acc
 */
float BoxShim::sqrt_expr(float y, float v, float ub, float amin, float d) {
  return (amin*d*d-2*v*d)*(amin*d*d-2*v*d) -
    4*(-d*d)*(2*amin*y-2*amin*ub + 2*amin*v*d-v*v);
}

/*
 * Computes the maximum allowed acceleration by the position shim.
 */
float BoxShim::max_acc_position(float y, float v, float ub, float amin) {
  float d = _d_ctrl*lookahead();
  float sexpr = sqrt_expr(y,v,ub,amin,d);
  if (sexpr >= 0) {
    float bound = (-(amin*d*d-2*v*d) - sqrt(sexpr))/(2*-d*d);
    if (-v/d <= bound) {
      return bound;
    } else if (0 < v && 2*(y-ub) != 0) {
      bound = v*v/(2*(y-ub));
      if (bound <= -v/d) {
	return bound;
      } else {
	return -v/d;
      }
    } else {
      return -v/d;
    }
  }
  return default_rect_action_one_dim(y, v, amin);
}

/*
 * Computes the maximum allowed acceleration by the velocity shim
 */
float BoxShim::max_acc_velocity(float v, float ub) {
  float d = _d_ctrl*lookahead();
  return (ub-v)/d;
}

bool BoxShim::ind_inv_acc(float y, float v, float ub, float amin) {
  if (0 <= v) {
    return y + sdist(v, amin) <= ub;
  } else {
    return y <= ub;
  }
}

bool BoxShim::ind_inv_vel(float v, float ub) {
  return v <= ub;
}

bool BoxShim::ind_inv(float vx, float vy, float x, float y, float ubx,
		      float ubvx, float uby, float ubvy) {
  return
    ind_inv_acc(x, vx, ubx, amin_X()) &&
    ind_inv_acc(-x, -vx, ubx, amin_X()) &&
    ind_inv_acc(y, vy, uby, amin_Y()) &&
    ind_inv_acc(-y, -vy, uby, amin_Y()) &&
    ind_inv_vel(vx, ubvx) &&
    ind_inv_vel(-vx, ubvx) &&
    ind_inv_vel(vy, ubvy) &&
    ind_inv_vel(-vy, ubvy);
}

bool BoxShim::can_run(state st) {
  bounds shifted_bounds = shift_bounds();
  state shifted_st = shift_state(st);
  return ind_inv(shifted_st.vx, shifted_st.vy, shifted_st.x, shifted_st.y,
		 shifted_bounds.ubx, shifted_bounds.ubvx, shifted_bounds.uby,
		 shifted_bounds.ubvy);
}

/*
 * Implements the actual monitor logic.
 */
monitor_check BoxShim::monitor_logic(float AX, float AY, float Theta, float vx,
				     float vy, float x, float y, float ubx,
				     float ubvx, float uby, float ubvy) {
  safety_check safe_check_x = safe_acc(AX, vx, x, ubx, ubvx, amin_X());
  bool safe_x = all_safe(safe_check_x);
  safety_check safe_check_y = safe_acc(AY, vy, y, uby, ubvy, amin_Y());
  bool safe_y = all_safe(safe_check_y);
  bool Theta_bound_check = roll_lb() <= Theta && Theta <= -roll_lb();

  monitor_check res;
  res.safe_x = safe_check_x;
  res.safe_y = safe_check_y;
  res.Theta_bound_check = Theta_bound_check;

  // Issue the proposed signal if it passes
  // all safety checks, otherwise issue the default
  if (safe_x && safe_y && Theta_bound_check) {
    res.ax = AX;
    res.ay = AY;

    res.cin.updated = false;
  } else if (safe_x && amin_X() <= AX && AX <= -amin_X()) {
    res.ax = AX;
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());

    res.cin = rect_to_polar(res.ay + gravity, AX);
    res.cin.updated = true;
  } else if (safe_y && amin_Y() <= AY && AY <= -amin_Y()) {
    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = AY;

    res.cin = rect_to_polar(AY + gravity, res.ax);
    res.cin.updated = true;
  } else {

    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());

    // We are rotated 90 degrees, so we reverse the arguments
    // We also account for gravity in the y direction
    res.cin = rect_to_polar(res.ay + gravity, res.ax);

    //    control_in res = default_action(x, y, vx, vy);
    res.cin.updated = true;
  }

  return res;
}

/*
 * Runs the monitor, returning a safe control input.
 */
control_in BoxShim::monitor(control_in proposed, state st) {

  bounds shifted_bounds = shift_bounds();
  state shifted_st = shift_state(st);

  float shifted_ubx = shifted_bounds.ubx;
  float shifted_ubvx = shifted_bounds.ubvx;
  float shifted_uby = shifted_bounds.uby;
  float shifted_ubvy = shifted_bounds.ubvy;

  float x = shifted_st.x;
  float y = shifted_st.y;
  float vx = shifted_st.vx;
  float vy = shifted_st.vy;
  float A = proposed.a;
  float Theta = proposed.theta;
  float AX = A*sin(Theta);
  float AY = A*cos(Theta)-gravity;

  monitor_check res;
  if (smooth()) {
    res.ax = constrain_float(AX, -max_acc_position(-x, -vx, shifted_ubx, amin_X()),
			     max_acc_position(x, vx, shifted_ubx, amin_X()));
    res.ax = constrain_float(res.ax, -max_acc_velocity(-vx, shifted_ubvx),
			     max_acc_velocity(vx, shifted_ubvx));
    res.ax = constrain_float(res.ax, amin_X(), -amin_X());
    res.ay = constrain_float(AY, -max_acc_position(-y, -vy, shifted_uby, amin_Y()),
			     max_acc_position(y, vy, shifted_uby, amin_Y()));
    res.ay = constrain_float(res.ay, -max_acc_velocity(-vy, shifted_ubvy),
			     max_acc_velocity(vy, shifted_ubvy));
    res.ay = constrain_float(res.ay, amin_Y(), -amin_Y());
    if (res.ax != AX || res.ay != AY) {
      res.cin = rect_to_polar(res.ay + gravity, res.ax);
      res.cin.updated = true;
    } else {
      res.cin = proposed;
      res.cin.updated = false;
    }
  } else {
    res = monitor_logic(AX, AY, Theta, vx, vy, x, y, shifted_ubx,
			shifted_ubvx, shifted_uby, shifted_ubvy);
    if (!res.cin.updated) {
      res.cin = proposed;
    }
  }

  // Set all the statistics
  _stats.x = st.x;
  _stats.y = st.y;
  _stats.vx = st.vx;
  _stats.vy = st.vy;
  _stats.A = A;
  _stats.Theta = Theta;
  _stats.a = res.cin.a;
  _stats.theta = res.cin.theta;
  _stats.AX = AX;
  _stats.AY = AY;
  _stats.ax = res.ax;
  _stats.ay = res.ay;
  _stats.amin_x = amin_X();
  _stats.amin_y = amin_Y();
  _stats.safe_x = all_safe(res.safe_x);
  _stats.safe_y = all_safe(res.safe_y);
  _stats.safe_x_vel_ub = res.safe_x.vel_ub;
  _stats.safe_x_vel_lb = res.safe_x.vel_lb;
  _stats.safe_x_pos_ub = res.safe_x.pos_ub;
  _stats.safe_x_pos_lb = res.safe_x.pos_lb;
  _stats.safe_y_vel_ub = res.safe_y.vel_ub;
  _stats.safe_y_vel_lb = res.safe_y.vel_lb;
  _stats.safe_y_pos_ub = res.safe_y.pos_ub;
  _stats.safe_y_pos_lb = res.safe_y.pos_lb;
  _stats.Theta_bound_check = res.Theta_bound_check;

  return res.cin;
}

state BoxShim::shift_state(state st) {
  state res;
  res.x = st.x+shift(x_ub(),x_lb());
  res.y = st.y+shift(y_ub(),y_lb());
  res.vx = st.vx+shift(vx_ub(),vx_lb());
  res.vy = st.vy+shift(vy_ub(),vy_lb());
  return res;
}

bounds BoxShim::shift_bounds() {
  bounds res;
  res.ubx = bound_shift(x_ub(),x_lb());
  res.ubvx = bound_shift(vx_ub(),vx_lb());
  res.uby = bound_shift(y_ub(),y_lb());
  res.ubvy = bound_shift(vy_ub(),vy_lb());
  return res;
}
