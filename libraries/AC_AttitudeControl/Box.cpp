#include "Box.h"
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"
#include "AP_Math.h"

extern const AP_HAL::HAL& hal;

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
 * Breaking acceleration in the X direction
 */
float BoxShim::amin_X() {
  return (amin_Z() + gravity) * tan(angle_lb());
}

/*
 * Breaking acceleration in the Y direction
 */
float BoxShim::amin_Y() {
  return (amin_Z() + gravity) * tan(angle_lb());
}

/*
 * Breaking acceleration in the Z direction
 */
float BoxShim::amin_Z() {
  return abraking();
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
polar BoxShim::rect_to_polar (float x, float y) {
  polar res;
  res.theta = atan2(y,x);
  res.a = sqrt((x*x) + (y*y));

  return res;
}

/*
 * Returns spherical coordinates such that A >= 0
 * -PI <= pitch <= PI and -PI/2 <= roll <= PI/2
 */
control_in BoxShim::rect_to_spherical (float x, float y, float z) {
  // Todo two rect to polar transformation, see wikipedia
  control_in res;
  polar zx = rect_to_polar(z, x);
  res.pitch = zx.theta;
  polar ay = rect_to_polar(zx.a, y);
  res.roll = ay.theta;
  res.a = ay.a;

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

bool BoxShim::ind_inv(float vx, float vy, float vz,
		      float x, float y, float z,
		      float ubx, float ubvx,
		      float uby, float ubvy,
		      float ubz, float ubvz) {
  return
    ind_inv_acc(x, vx, ubx, amin_X()) &&
    ind_inv_acc(-x, -vx, ubx, amin_X()) &&
    ind_inv_acc(y, vy, uby, amin_Y()) &&
    ind_inv_acc(-y, -vy, uby, amin_Y()) &&
    ind_inv_acc(z, vz, ubz, amin_Z()) &&
    ind_inv_acc(-z, -vz, ubz, amin_Z()) &&
    ind_inv_vel(vx, ubvx) &&
    ind_inv_vel(-vx, ubvx) &&
    ind_inv_vel(vy, ubvy) &&
    ind_inv_vel(-vy, ubvy) &&
    ind_inv_vel(vz, ubvz) &&
    ind_inv_vel(-vz, ubvz);
}

bool BoxShim::can_run(state st) {
  bounds shifted_bounds = shift_bounds();
  state shifted_st = shift_state(st);
  return ind_inv(shifted_st.vx, shifted_st.vy, shifted_st.vz,
		 shifted_st.x, shifted_st.y, shifted_st.z,
		 shifted_bounds.ubx, shifted_bounds.ubvx,
		 shifted_bounds.uby, shifted_bounds.ubvy,
		 shifted_bounds.ubz, shifted_bounds.ubvz);
}

/*
 * Implements the actual monitor logic.
 * TODO - add z to this function
 */
monitor_check BoxShim::monitor_logic(float AX, float AY, float AZ, float Pitch, float Roll,
				     float vx, float vy, float vz, float x, float y, float z,
				     float ubx, float ubvx, float uby, float ubvy,
				     float ubz, float ubvz) {
  safety_check safe_check_x = safe_acc(AX, vx, x, ubx, ubvx, amin_X());
  bool safe_x = all_safe(safe_check_x);
  safety_check safe_check_y = safe_acc(AY, vy, y, uby, ubvy, amin_Y());
  bool safe_y = all_safe(safe_check_y);
  safety_check safe_check_z = safe_acc(AZ, vz, z, ubz, ubvz, amin_Z());
  bool safe_z = all_safe(safe_check_z);
  bool Roll_bound_check = angle_lb() <= Roll && Roll <= -angle_lb();
  bool Pitch_bound_check = angle_lb() <= Pitch && Pitch <= -angle_lb();

  monitor_check res;
  res.safe_x = safe_check_x;
  res.safe_y = safe_check_y;
  res.safe_z = safe_check_z;
  res.Roll_bound_check = Roll_bound_check;
  res.Pitch_bound_check = Pitch_bound_check;

  // Issue the proposed signal if it passes
  // all safety checks, otherwise issue the default
  if (safe_x && safe_y && safe_z && Roll_bound_check && Pitch_bound_check) {
    res.ax = AX;
    res.ay = AY;
    res.az = AZ;

    res.cin.updated = false;
  } else if (safe_x && safe_y && amin_X() <= AX && AX <= -amin_X()
	     && amin_Y() <= AY && AY <= -amin_Y()) {
    res.ax = AX;
    res.ay = AY;
    res.az = default_rect_action_one_dim(z, vz, amin_Z());

    // hal.console->printf("Violated Z\n");
    // hal.console->printf("(AX,AY,AZ): (%f,%f,%f)\n", res.ax, res.ay, res.az + gravity);
    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else if (safe_y && safe_z && amin_Y() <= AY && AY <= -amin_Y()
	     && amin_Z() <= AZ && AZ <= -amin_Z()) {
    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = AY;
    res.az = AZ;

    // hal.console->printf("Violated X\n");
    // hal.console->printf("(AX,AY,AZ): (%f,%f,%f)\n", res.ax, res.ay, res.az + gravity);
    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else if (safe_x && safe_z && amin_X() <= AX && AX <= -amin_X()
	     && amin_Z() <= AZ && AZ <= -amin_Z()) {
    res.ax = AX;
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());
    res.az = AZ;

    // hal.console->printf("Violated Y\n");
    // hal.console->printf("(AX,AY,AZ): (%f,%f,%f)\n", res.ax, res.ay, res.az + gravity);
    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else if (safe_x && amin_X() <= AX && AX <= -amin_X()) {
    res.ax = AX;
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());
    res.az = default_rect_action_one_dim(z, vz, amin_Z());

    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else if (safe_y && amin_Y() <= AY && AY <= -amin_Y()) {
    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = AY;
    res.az = default_rect_action_one_dim(z, vz, amin_Z());

    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else if (safe_z && amin_Z() <= AZ && AZ <= -amin_Z()) {
    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());
    res.az = AZ;

    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  } else {
    res.ax = default_rect_action_one_dim(x, vx, amin_X());
    res.ay = default_rect_action_one_dim(y, vy, amin_Y());
    res.az = default_rect_action_one_dim(z, vz, amin_Z());

    res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
    res.cin.updated = true;
  }

  return res;
}

/*
 * Runs the monitor, returning a safe control input.
 */
monitor_check BoxShim::monitor(control_in proposed, state st) {

  bounds shifted_bounds = shift_bounds();
  state shifted_st = shift_state(st);

  float shifted_ubx = shifted_bounds.ubx;
  float shifted_ubvx = shifted_bounds.ubvx;
  float shifted_uby = shifted_bounds.uby;
  float shifted_ubvy = shifted_bounds.ubvy;
  float shifted_ubz = shifted_bounds.ubz;
  float shifted_ubvz = shifted_bounds.ubvz;

  float x = shifted_st.x;
  float y = shifted_st.y;
  float z = shifted_st.z;
  float vx = shifted_st.vx;
  float vy = shifted_st.vy;
  float vz = shifted_st.vz;
  float A = proposed.a;
  float Roll = proposed.roll;
  float Pitch = proposed.pitch;
  float AX = A*cos(Pitch)*sin(Roll);
  float AY = A*sin(Pitch);
  float AZ = A*cos(Pitch)*cos(Roll)-gravity;

  monitor_check res;
  res.AX = AX;
  res.AY = AY;
  res.AZ = AZ;
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
    res.az = constrain_float(AZ, -max_acc_position(-z, -vz, shifted_ubz, amin_Z()),
			     max_acc_position(z, vz, shifted_ubz, amin_Z()));
    res.az = constrain_float(res.az, -max_acc_velocity(-vz, shifted_ubvz),
			     max_acc_velocity(vz, shifted_ubvz));
    res.az = constrain_float(res.az, amin_Z(), -amin_Z());
    if (res.ax != AX || res.ay != AY || res.az != AZ) {
      // Todo new transformation required, involving Z
      res.cin = rect_to_spherical(res.ax, res.ay, res.az + gravity);
      res.cin.updated = true;
    } else {
      res.cin = proposed;
      res.cin.updated = false;
    }
  } else {
    res = monitor_logic(AX, AY, AZ, Pitch, Roll, vx, vy, vz, x, y, z,
			shifted_ubx, shifted_ubvx, shifted_uby, shifted_ubvy,
			shifted_ubz, shifted_ubvz);

    if (!res.cin.updated) {
      res.cin = proposed;
    }
  }

  return res;
}

state BoxShim::shift_state(state st) {
  state res;
  res.x = st.x+shift(x_ub(),x_lb());
  res.y = st.y+shift(y_ub(),y_lb());
  res.z = st.z+shift(z_ub(),z_lb());
  res.vx = st.vx+shift(vx_ub(),vx_lb());
  res.vy = st.vy+shift(vy_ub(),vy_lb());
  res.vz = st.vz+shift(vz_ub(),vz_lb());
  return res;
}

bounds BoxShim::shift_bounds() {
  bounds res;
  res.ubx = bound_shift(x_ub(),x_lb());
  res.ubvx = bound_shift(vx_ub(),vx_lb());
  res.uby = bound_shift(y_ub(),y_lb());
  res.ubvy = bound_shift(vy_ub(),vy_lb());
  res.ubz = bound_shift(z_ub(),z_lb());
  res.ubvz = bound_shift(vz_ub(),vz_lb());
  return res;
}
