#ifndef Boxes_H
#define Boxes_H

#include <map>
#include <math.h>
#include <AP_Math.h>
#include <vectorN.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"
#include "Box.h"

struct shim_stats {
  std::map<uint8_t, bool> can_run;
  float x;
  float y;
  float vx;
  float vy;
  uint16_t throttle;
  uint8_t angle_boost;
  float A_proposed;
  float Theta_proposed;
  float a;
  float theta;
};

class BoxesShim : public AC_AttitudeShim {
 public:
  BoxesShim(AP_AHRS &ahrs,
	  const AP_Vehicle::MultiCopter &aparm,
	  AP_Motors& motors,
	  AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
	  AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
          float mid_throttle, const AP_InertialNav& inav
	  ) :
  AC_AttitudeShim(ahrs, aparm, motors, pi_angle_roll, pi_angle_pitch,
		  pi_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
    _mid_throttle(mid_throttle),
    _inav(inav)
    {
    };

  float get_x();
  float get_y();
  float get_vx();
  float get_vy();

  void add_box(const uint8_t id, const float h_ub, const float h_lb, const float hprime_ub, const float hprime_lb,
	       const float x_ub, const float x_lb, const float xprime_ub, const float xprime_lb,
	       const float roll_lb, const float abraking, const bool smooth, const float lookahead,
	       const float d_ctrl);
  void clear_boxes();

  BoxShim get_box(const uint8_t id);
  bool has_box(const uint8_t id);

  // enable/disable shim
  void enable_shim() {_shim_on = true;}
  void disable_shim() {_shim_on = false;}

  void set_mid_throttle(float t) {_mid_throttle = t;}

  // get shim information
  bool shim_on() {return _shim_on;}
  float mid_throttle() {return _mid_throttle;}

  shim_stats get_shim_stats() {return _stats;}

 private:

  void attitude_shim_entry_point(Att_shim_params params, bool first_call);

  float throttle_to_accel_scale();
  float get_acc_from_throttle(float throttle);
  float get_throttle_from_acc(float A);

  std::map<uint8_t, BoxShim> _boxes;

  // for actual shim
  // is the shim on?
  bool _shim_on;
  float _mid_throttle;

  shim_stats _stats;

  uint8_t _last_shim_id;

  const AP_InertialNav& _inav;
};

#endif //Boxes_H
