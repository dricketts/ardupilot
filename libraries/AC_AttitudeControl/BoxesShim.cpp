#include "BoxesShim.h"

float BoxesShim::get_x() {
  return _inav.get_position().y;
}

float BoxesShim::get_y() {
  return _inav.get_position().z;
}

float BoxesShim::get_vx() {
  return _inav.get_velocity().y;
}

float BoxesShim::get_vy() {
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

  _stats.x = st.x;
  _stats.y = st.y;
  _stats.vx = st.vx;
  _stats.vy = st.vy;
  _stats.throttle = throttle;
  _stats.angle_boost = params.angle_boost;
  _stats.A_proposed = proposed.a;
  _stats.Theta_proposed = proposed.theta;
  float a;
  float theta;

  for (std::map<uint8_t,BoxShim>::iterator it = _boxes.begin(); it != _boxes.end(); ++it) {
    BoxShim box = it->second;
    bool can_run = box.can_run(st);
    _stats.can_run[it->first] = can_run;

    if (can_run) {
      control_in safe = box.monitor(proposed, st);

      if (shim_on() && safe.updated) {
	params.roll = degrees(safe.theta)*100.0f;
	params.throttle = get_throttle_from_acc(safe.a);
	params.angle_boost = false;
      }

      break;
    }
  }

  _stats.theta = radians(wrap_180_cd_float(params.roll)/100.0f);
  _stats.a =
    get_acc_from_throttle(params.angle_boost ?
			  get_angle_boost(params.throttle + 0.0f) :
			  params.throttle + 0.0f);

  AC_AttitudeShim::attitude_shim_entry_point(params, first_call);
}

void BoxesShim::add_box(const uint8_t id, const float y_ub, const float y_lb, const float vy_ub, const float vy_lb,
			const float x_ub, const float x_lb, const float vx_ub, const float vx_lb,
			const float roll_lb, const float abraking, const bool smooth, const float lookahead) {

    BoxShim box;
    box.set_y_ub(y_ub);
    box.set_y_lb(y_lb);
    box.set_vy_ub(vy_ub);
    box.set_vy_lb(vy_lb);
    box.set_x_ub(x_ub);
    box.set_x_lb(x_lb);
    box.set_vx_ub(vx_ub);
    box.set_vx_lb(vx_lb);
    box.set_roll_lb(roll_lb);
    box.set_abraking(abraking);
    box.set_lookahead(lookahead);
    box.set_d_ctrl(_d_ctrl);
    box.set_smooth(smooth);
    
    _boxes[id] = box;
}

void BoxesShim::clear_boxes() {
  _boxes.clear();
}

BoxShim BoxesShim::get_box(const uint8_t id) {
  return _boxes[id];
}

bool BoxesShim::has_box(const uint8_t id) {
  return _boxes.count(id) == 1;
}
