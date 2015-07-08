#include "Box.h"

/*
 * Distance traveled with initial velocity V,
 * acceleration a, for time t
 */
static float tdist(float V, float a, float t) {
    return (V*t) + ((a*t*t)/2);
}

/*
 * Distance required to stop when traveling
 * at positive velocity V
 */
static float sdist(float V, float amin) {
    return -(V*V)/(2*amin);
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
  return (amin_X() + g) * tan(_params.theta_min);
}

/*
 * The safety check for the position monitor.
 */
bool BoxShim::pos_safe_acc(float a, float v, float y,
			   float ub, float amin) {
  float mxa = std::max(a,0.0f);
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
control_in BoxShim::default_action(state st) {
  float ax = default_rect_action_one_dim(st.x, st.vx,
					 amin_X());
  float ay = default_rect_action_one_dim(st.y, st.vy,
					 amin_Y());

  // We are rotated 90 degrees, so we reverse the arguments
  // We also account for gravity in the y direction
  return rect_to_polar(ay + g, ax);
}

static float shift(float ub, float lb) {
  return -((lb + ub)/2);
}

static float bound_shift(float ub, float lb) {
  return (ub - lb)/2;
}

/*
 * Runs the safety check in one dimension
 */
bool BoxShim::safe_acc(float a, float v, float y,
		       float ub, float ubv, float amin) {
  return
    vel_safe_acc(-a,-v,ubv) && vel_safe_acc(a,v,ubv) &&
    pos_safe_acc(a,v,y,ub,amin) &&
    pos_safe_acc(-a,-v,-y,ub,amin);
}

/*
 * Runs the monitor, returning a safe control input.
 */
control_in BoxShim::monitor(control_in proposed, state st) {

  float x = st.x;
  float y = st.y;
  float vx = st.vx;
  float vy = st.vy;
  float A = proposed.a;
  float Theta = proposed.theta;

  float ubx = _params.ubx;
  float lbx = _params.lbx;
  float uby = _params.uby;
  float lby = _params.lby;
  float ubvx = _params.ubvx;
  float lbvx = _params.lbvx;
  float ubvy = _params.ubvy;
  float lbvy = _params.lbvy;

  // Issue the proposed signal if it passes
  // all safety checks, otherwise issue the default
  if (safe_acc(A*sin(Theta), vx+shift(ubvx,lbvx),
	       x+shift(ubx,lbx), bound_shift(ubx,lbx),
	       bound_shift(ubvx,lbvx), amin_X()) &&
      safe_acc(A*cos(Theta)-g, vy+shift(ubvy,lbvy),
	       y+shift(uby,lby), bound_shift(uby,lby),
	       bound_shift(ubvy,lbvy), amin_Y()) &&
      _params.theta_min <= Theta <= -_params.theta_min) {
    return proposed;
  } else {
    return default_action(st);
  }
}
