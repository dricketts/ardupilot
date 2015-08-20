// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeShim_H
#define AC_AttitudeShim_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <AP_Motors.h>
#include <DataFlash.h>
#include <AC_PID.h>
#include <AC_P.h>

// This implements a shim over the quadcopter attitude controller
// Enabling run-time monitoring at the level of abstraction of sending
// Throttle/attitude commands.

// Enumeration of the functions we shim over (used for dispatching)
enum Att_shim_fn {ATT_SET_SMOOTH, ATT_SET, ATT_SET_SLEW, ATT_SET_RATES, ATT_SET_BODYFRAME, THROTTLE_SET};

// statistics about the shim and motors
// should be averages over the number of
// iterations since the last time this struct
// was requested
// we'll have a variable for count, a
// variable for the sum of each field,
// and we'll take the average when the
// stats are requested
// we'll also reset everything to zero
// when the stats are requested
struct shim_stats {
	bool something_weird;
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


// Data structure used for passing parameters to shim functions
// This is NOT the parameters data structure for MAVLink messaging.

struct Att_shim_params {
  Att_shim_fn which_fn;
  // pitch, roll, yaw parameters - have different semantics depending on which function
  float roll;
  float pitch;
  float yaw;
  // extra options for attitude
  float smoothing_gain;
  bool slew_yaw;
  // throttle parameters
  int16_t throttle;
  bool angle_boost;
  // used when storing parameters in static variables
  bool initialized;
};

// @class     AC_AttitudeShim
class AC_AttitudeShim : public AC_AttitudeControl {
public:
  // Constructor
  // Takes parameters for the superclass,
  // As well as initial values for bounds and other internal shim variables
  // TODO - DELAY...
  // NB - currently d_ctrl is the only one of these that cannot be set
  // from the ground station
  AC_AttitudeShim( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_Motors& motors,
                        AC_P& pi_angle_roll, AC_P& pi_angle_pitch, AC_P& pi_angle_yaw,
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw,
                        const float h_ub, const float h_lb, const float hprime_ub, const float hprime_lb,
                        const float x_ub, const float x_lb, const float xprime_ub, const float xprime_lb,
				   const float roll_lb, const float abraking, const float mid_throttle,
				   const float d_ctrl, const float lookahead) :
                        AC_AttitudeControl(ahrs, aparm, motors, pi_angle_roll, pi_angle_pitch,
                                           pi_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
                        _h_ub(h_ub), _h_lb(h_lb), _hprime_ub(hprime_ub), _hprime_lb(hprime_lb),
                        _x_ub(x_ub), _x_lb(x_lb), _xprime_ub(xprime_ub), _xprime_lb(xprime_lb),
                   	  _roll_lb(roll_lb), _abraking(abraking), _mid_throttle(mid_throttle),
                	  _d_ctrl(d_ctrl), _lookahead(lookahead)
    {
    };

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeShim() {}

    //
    // Methods to be called by upper controllers to request and implement desired attitude
    // (Overrides)
    //

    // angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
    //      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
    void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);

    // angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
    void angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef);

    // angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
    //  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the YAW_SLEW parameter
    void angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw);

    // rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
    void rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef);

    // rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
    void rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf);

    //
    // throttle functions
    // (Overrides)
    //

    // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
    // provide 0 to cut motors
    void set_throttle_out(int16_t throttle_pwm, bool apply_angle_boost);

    //
    // Accessors
    //

    // enable/disable shim
    void enable_shim() {_shim_on = true;}
    void disable_shim() {_shim_on = false;}

	void set_smooth(bool smooth) {_smooth = smooth;}

	// Indicate whether something weird has happened
	void something_weird(bool weird) {_stats.something_weird = weird;}

    // set shim bounding parameters
    void set_h_ub(float b) {_h_ub = b;}
    void set_h_lb(float b) {_h_lb = b;}
    void set_hprime_ub(float b) {_hprime_ub = b;}
    void set_hprime_lb(float b) {_hprime_lb = b;}
    void set_x_ub(float b) {_x_ub = b;}
    void set_x_lb(float b) {_x_lb = b;}
    void set_xprime_ub(float b) {_xprime_ub = b;}
    void set_xprime_lb(float b) {_xprime_lb = b;}
    void set_roll_lb(float r) {_roll_lb = r;}
    void set_abraking(float a) {_abraking = a;}
	void set_mid_throttle(float t) {_mid_throttle = t;}
	void set_lookahead(float x) {_lookahead = x;}

    // get shim information
    bool shim_on() {return _shim_on;}
	bool smooth() {return _smooth;}
    float h_ub() {return _h_ub;}
    float h_lb() {return _h_lb;}
    float hprime_ub() {return _hprime_ub;}
    float hprime_lb() {return _hprime_lb;}
    float x_ub() {return _x_ub;}
    float x_lb() {return _x_lb;}
    float xprime_ub() {return _xprime_ub;}
    float xprime_lb() {return _xprime_lb;}
    float roll_lb() {return _roll_lb;}
    float abraking() {return _abraking;}
	float mid_throttle() {return _mid_throttle;}
	float lookahead() {return _lookahead;}

    //
    // Stats Reporing (Accessor)
    //
    shim_stats get_shim_stats() {return _stats;}

protected:

  // Main entry point for the shim. All the overriden throttle and attitude functions
  // call into this one.
  // params stores information about the function to call, and its arguments
  // first_call stores whether we should also update the throttle if we are
  // currently updating attitude (or vice versa)

  //
  // Member Variables
  //

  // for actual shim
  // is the shim on?
  bool _shim_on;
  bool _smooth;
  // delay associated with actuators
  const float _d_ctrl;
  // height bounds
  float _h_ub;
  float _h_lb;
  // vertical velocity bounds
  float _hprime_ub;
  float _hprime_lb;
  // horizontal bounds
  float _x_ub;
  float _x_lb;
  // horizontal velocity bounds
  float _xprime_ub;
  float _xprime_lb;
  // roll bounds
  float _roll_lb;
  // braking acceleration for vertical dimension
  float _abraking;
  float _mid_throttle;
  float _lookahead;

  shim_stats _stats;

  // Entry point of shim; shimmed-over functions call into this
  virtual void attitude_shim_entry_point(Att_shim_params params, bool first_call);
};

#endif //AC_AttitudeShim_H
