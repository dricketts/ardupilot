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
                        AC_PID& pid_rate_roll, AC_PID& pid_rate_pitch, AC_PID& pid_rate_yaw) :
                        AC_AttitudeControl(ahrs, aparm, motors, pi_angle_roll, pi_angle_pitch,
                                           pi_angle_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw)
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

protected:

  // Main entry point for the shim. All the overriden throttle and attitude functions
  // call into this one.
  // params stores information about the function to call, and its arguments
  // first_call stores whether we should also update the throttle if we are
  // currently updating attitude (or vice versa)

  //
  // Member Variables
  //

  // Entry point of shim; shimmed-over functions call into this
  virtual void attitude_shim_entry_point(Att_shim_params params, bool first_call);
};

#endif //AC_AttitudeShim_H
