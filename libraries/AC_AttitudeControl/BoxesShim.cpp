#include "BoxesShim.h"

/*
 * Here is the derivation for this formula:
 *
 * (x - x')^2 + (y - y')^2 + (z - z')^2
 * x^2 - 2xx' + x'^2 + y^2 - 2yy' + y'^2 + z^2 - 2zz' + z'^2
 * r1^2 + r2^2 - 2*(xx' + yy' + zz')
 * r1^2 + r2^2 - 2*(r1*cos(roll)*sin(pitch)*r2*cos(roll')*sin(pitch') +
 *                  r1*sin(roll)*r2*sin(roll') +
 *                  r1*cos(roll)*cos(pitch)*r2*cos(roll')*cos(pitch'))
 * r1^2 + r2^2 - 2*r1*r2(cos(roll)*sin(pitch)*cos(roll')*sin(pitch') +
 *                       sin(roll)*sin(roll') +
 *                       cos(roll)*cos(pitch)*cos(roll')*cos(pitch'))
 * r1^2 + r2^2 - 2*r1*r2(cos(roll)*cos(roll')*(sin(pitch)*sin(pitch') + cos(pitch)*cos(pitch')) +
 *                       sin(roll)*sin(roll'))
 * r1^2 + r2^2 - 2*r1*r2(cos(roll)*cos(roll')*(cos(pitch - pitch')) +
 *                       sin(roll)*sin(roll'))
 */
static float cin_diff(control_in cin1, control_in cin2) {
  float r1 = cin1.a;
  float r2 = cin2.a;
  return r1 * r1 + r2 * r2 - 2 * r1 * r2 *
    (cos(cin1.roll)*cos(cin2.roll)*cos(cin1.pitch - cin2.pitch) +
     sin(cin1.roll)*sin(cin2.roll));
}

float BoxesShim::get_x() {
  return _inav.get_position().x;
}

float BoxesShim::get_y() {
  return _inav.get_position().y;
}

float BoxesShim::get_z() {
  return _inav.get_position().z;
}

float BoxesShim::get_vx() {
  return _inav.get_velocity().x;
}

float BoxesShim::get_vy() {
  return _inav.get_velocity().y;
}

float BoxesShim::get_vz() {
  return _inav.get_velocity().z;
}

float BoxesShim::throttle_to_accel_scale() {
  return gravity/mid_throttle();
}

// Gives an estimate of an upper bound on acceleration
// using the throttle value.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float BoxesShim::get_acc_from_throttle(float throttle) {
  return throttle*throttle_to_accel_scale();
}

float BoxesShim::get_throttle_from_acc(float A) {
  return A/throttle_to_accel_scale();
}

void BoxesShim::attitude_shim_entry_point(Att_shim_params params, bool first_call) {
  
  //_params = {-500.0f, 10000.0f, -10000.0f, 6000.0f, -2000.0f, 500.0f, -500.0f, 500.0f, -500.0f, -(M_PI + 0.0f)/4.0f };

  static float roll = 0.0f;
  static float pitch = 0.0f;
  static int16_t throttle = 0;

  switch (params.which_fn) {

  case THROTTLE_SET:

    throttle = params.throttle;

    break;
    
  default:

    roll = params.roll;
    pitch = params.pitch;

  }

  control_in proposed;
  proposed.roll = radians(wrap_180_cd_float(roll)/100.0f);
  proposed.pitch = radians(wrap_180_cd_float(pitch)/100.0f);
  proposed.a =
    get_acc_from_throttle(params.angle_boost ?
			  get_angle_boost(throttle + 0.0f) :
			  throttle + 0.0f);

  state st;
  st.x = get_x();
  st.y = get_y();
  st.z = get_z();
  st.vx = get_vx();
  st.vy = get_vy();
  st.vz = get_vz();

  _stats.x = st.x;
  _stats.y = st.y;
  _stats.z = st.z;
  _stats.vx = st.vx;
  _stats.vy = st.vy;
  _stats.vz = st.vz;
  _stats.throttle = throttle;
  _stats.angle_boost = params.angle_boost;
  _stats.A_proposed = proposed.a;
  _stats.Roll_proposed = proposed.roll;
  _stats.Pitch_proposed = proposed.pitch;

  bool shim_engaged = false;
  control_in actual;
  bool first = true;

  for (uint8_t i = 1; i <= _boxes.size(); i++) {
    if (_boxes.has_box(i)) {
      BoxShim box = _boxes.get(i);
      bool can_run = box.can_run(st);
      set_can_run(i, can_run);
    
      if (can_run) {
	shim_engaged = true;
      
	control_in check = box.monitor(proposed, st);
	if (first || cin_diff(check, proposed) < cin_diff(actual, proposed)) {
	  actual = check;
	  first = false;
	  _last_shim_id = i;
	}
      }
    }
  }
  
  // None of the shims are enabled, we default to the
  // last shim that was used.
  if (!shim_engaged && has_box(_last_shim_id)) {
    // run the last shim that intervened
    actual = _boxes.get(_last_shim_id).monitor(proposed, st);
    shim_engaged = true;
  }

  // If the shim is on, and it engaged then we need to set
  // the values.
  if (shim_on() && shim_engaged) {
      params.roll = degrees(actual.roll)*100.0f;
      params.pitch = degrees(actual.pitch)*100.0f;
      params.throttle = get_throttle_from_acc(actual.a);
      params.angle_boost = false;
  }

  // Statistics reporting
  _stats.roll = radians(wrap_180_cd_float(params.roll)/100.0f);
  _stats.pitch = radians(wrap_180_cd_float(params.pitch)/100.0f);
  _stats.a =
    get_acc_from_throttle(params.angle_boost ?
			  get_angle_boost(params.throttle + 0.0f) :
			  params.throttle + 0.0f);

  AC_AttitudeShim::attitude_shim_entry_point(params, first_call);
}

void BoxesShim::add_box(const uint8_t id,
			const float ubx, const float lbx, const float ubvx, const float lbvx,
			const float uby, const float lby, const float ubvy, const float lbvy,
			const float ubz, const float lbz, const float ubvz, const float lbvz,
			const float angle_lb, const float abraking, const bool smooth, const float lookahead,
			const float d_ctrl) {

    BoxShim box;
    box.set_x_ub(ubx);
    box.set_x_lb(lbx);
    box.set_vx_ub(ubvx);
    box.set_vx_lb(lbvx);
    box.set_y_ub(uby);
    box.set_y_lb(lby);
    box.set_vy_ub(ubvy);
    box.set_vy_lb(lbvy);
    box.set_z_ub(ubz);
    box.set_z_lb(lbz);
    box.set_vz_ub(ubvz);
    box.set_vz_lb(lbvz);
    box.set_angle_lb(angle_lb);
    box.set_abraking(abraking);
    box.set_lookahead(lookahead);
    box.set_d_ctrl(d_ctrl);
    box.set_smooth(smooth);
    
    _boxes.set(id, box);
}

void BoxesShim::clear_boxes() {
  _boxes.clear();
}

BoxShim BoxesShim::get_box(const uint8_t id) {
  return _boxes.get(id);
}

bool BoxesShim::has_box(const uint8_t id) {
  return _boxes.has_box(id);
}

void BoxesShim::set_can_run(uint8_t id, bool can_run) {
  if (id == 1) {
    _stats.can_run1 = can_run;
  } else if (id == 2) {
    _stats.can_run2 = can_run;
  } else if (id == 3) {
    _stats.can_run3 = can_run;
  } else if (id == 4) {
    _stats.can_run4 = can_run;
  }

}
