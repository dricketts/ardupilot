// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_HAL.h>
#include "AC_AttitudeControl.h"
#include "AC_AttitudeShim.h"

extern const AP_HAL::HAL& hal;

// for use within attitude_shim_entry_point
#define PRINTERVAL 20
#define iprintf(...) if(printerval == 0) hal.console->printf(__VA_ARGS__)

// Attitude shim implementation

// Shim entry point
// Params: stores a description of which attitude/throttle function to call out to
// first_call: if true, call the throttle controller after making an attitude update (or vice versa)
// ("structurally recursive" in first_call)
void AC_AttitudeShim::attitude_shim_entry_point(Att_shim_params params, bool first_call) {
  // for testing
  // first_call = false;

  // ensure we don't print too often
  static int printerval = -1;
  printerval++;
  if (printerval == PRINTERVAL) printerval = 0;

  // Keep around two old sets of params:
  // One for most recently called attitude function
  static Att_shim_params last_attitude_call = {
    ATT_SET_SMOOTH, // which_fn
    0.0, 0.0, 0.0, // roll, pitch, yaw
    0.0, // smoothing_gain
    false, // slew_yaw
    0, // throttle
    false, // angle_boost
    false // NOT yet initialized
  };

  // One for most recently called throttle function
  static Att_shim_params last_throttle_call = {
    THROTTLE_SET, // which_fn
    0.0, 0.0, 0.0, // roll, pitch, yaw
    0.0, // smoothing_gain
    false, // slew_yaw
    0, // throttle
    false, // angle_boost
    false // NOT yet initialized
  };



  // track stats on the last attitude/throttle calls
  static int attitude_calls = 0;
  static int throttle_calls = 0;

  if (params.which_fn == THROTTLE_SET) {
    //iprintf("Throttle set to %d\n", params.throttle);
    throttle_calls++;
    //iprintf("Total throttle calls: %d\n", throttle_calls);
    //iprintf("Total attitude calls: %d\n\n", attitude_calls);
  }
  else {
    //iprintf("(Roll, Pitch, Yaw) set to (%f, %f, %f)\n", params.roll, params.pitch, params.yaw);
    attitude_calls++;
    //iprintf("Total throttle calls: %d\n", throttle_calls);
    //iprintf("Total attitude calls: %d\n\n", attitude_calls);
  }

  // Call the function that the user wanted.
  // Then, if the function was throttle, see if the user previously called
  // An attitude function (or vice versa). If they did, re-issue that same call.
  // This is to ensure throttle and attitude are updated at same frequency
  switch (params.which_fn) {
    case ATT_SET_SMOOTH :
      AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw_smooth(params.roll, params.pitch, params.yaw, params.smoothing_gain);
      if(last_throttle_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_throttle_call, false);
      }
      last_attitude_call = params;
      break;

    case ATT_SET :
      AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw(params.roll, params.pitch, params.yaw);
      if(last_throttle_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_throttle_call, false);
      }
      last_attitude_call = params;
      break;

    case ATT_SET_SLEW :
      AC_AttitudeControl::angle_ef_roll_pitch_yaw(params.roll, params.pitch, params.yaw, params.slew_yaw);
      if(last_throttle_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_throttle_call, false);
      }
      last_attitude_call = params;
      break;

    case ATT_SET_RATES :
      AC_AttitudeControl::rate_ef_roll_pitch_yaw(params.roll, params.pitch, params.yaw);
      if(last_throttle_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_throttle_call, false);
      }
      last_attitude_call = params;
      break;

    case ATT_SET_BODYFRAME:
      AC_AttitudeControl::rate_bf_roll_pitch_yaw(params.roll, params.pitch, params.yaw);
      if(last_throttle_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_throttle_call, false);
      }
      last_attitude_call = params;
      break;

    case THROTTLE_SET:
      AC_AttitudeControl::set_throttle_out(params.throttle, params.angle_boost);
      if(last_attitude_call.initialized && first_call){
        AC_AttitudeShim::attitude_shim_entry_point(last_attitude_call, false);
      }
      last_throttle_call = params;

      break;
  }
}

//
// Overrides (call into entry point)
//

// angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
//      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
void AC_AttitudeShim::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain) {
  Att_shim_params params;

  params.which_fn = ATT_SET_SMOOTH;
  params.roll = roll_angle_ef;
  params.pitch = pitch_angle_ef;
  params.yaw = yaw_rate_ef;
  params.smoothing_gain = smoothing_gain;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}

// angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
void AC_AttitudeShim::angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef) {
  Att_shim_params params;

  params.which_fn = ATT_SET;
  params.roll = roll_angle_ef;
  params.pitch = pitch_angle_ef;
  params.yaw = yaw_rate_ef;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}

// angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
//  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the YAW_SLEW parameter
void AC_AttitudeShim::angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw){
  Att_shim_params params;

  params.which_fn = ATT_SET_SLEW;
  params.roll = roll_angle_ef;
  params.pitch = pitch_angle_ef;
  params.yaw = yaw_angle_ef;
  params.slew_yaw = slew_yaw;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}

// rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
void AC_AttitudeShim::rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef){
  Att_shim_params params;

  params.which_fn = ATT_SET_RATES;
  params.roll = roll_rate_ef;
  params.pitch = pitch_rate_ef;
  params.yaw = yaw_rate_ef;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}

// rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
void AC_AttitudeShim::rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf){
  Att_shim_params params;

  params.which_fn = ATT_SET_BODYFRAME;
  params.roll = roll_rate_bf;
  params.pitch = pitch_rate_bf;
  params.yaw = yaw_rate_bf;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}

// set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
// provide 0 to cut motors
void AC_AttitudeShim::set_throttle_out(int16_t throttle_pwm, bool apply_angle_boost){
  Att_shim_params params;

  params.which_fn = THROTTLE_SET;
  params.throttle = throttle_pwm;
  params.angle_boost = apply_angle_boost;
  params.initialized = true;

  AC_AttitudeShim::attitude_shim_entry_point(params, true);
}
