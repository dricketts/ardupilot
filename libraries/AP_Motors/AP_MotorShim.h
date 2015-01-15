// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorShim.h
/// @brief	Motor shim class for Quadcopters

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include "AP_MotorsQuad.h"    // Parent Motors Quad library

// This is an upper bound height shim
// This means that the shim takes a proposed
// array of control signals to the motors and
// decides whether they are safe. If they are
// safe, the shim sends exactly those control
// signals. Otherwise, it sends the minimum
// control signal to the motors.
// The safety check and the safe value are
// such that the height of the system will
// always stay below the upper bound.

// Ratio between pwm of a single motor
// (scaled to be between 0 and 1) and the thrust
// it produces. Equal to (g/(4*hover_throttle)).
// Taken from python simulation in
// Tools/autotest/pysim/multicopter.py.
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
  static const float pwm_accel_scale = 5.448138888889;
#endif
// Maximum possible acceleration.
static const float amax = 4 * pwm_accel_scale;
// Acceleration due to gravity in cm/s/s.
// Sensors seem to use cm as length unit.
// This constant has same level of precision
// as python simulation.
static const float amin = -980.665;

/// @class      AP_MotorShim
class AP_MotorShim : public AP_MotorsQuad {
public:

    /// Constructor
    // In addition to the arguments required by the parent class,
    // this constructor takes an altitude upper bound and a delay d
    // which gives the maximum time between updates of the shim
    // using new altitude and vertical velocity estimates.
    AP_MotorShim( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw,
                  const AP_InertialNav_NavEKF& nav, const float ub, const float d,
                  uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT)
        : AP_MotorsQuad(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz), _inertial_nav(nav),
          _ub(ub), _d(d), _a(0) {
    };

protected:
    // output - sends commands to the motors
    virtual void        output_armed();

private:
    // OneDimAccShim1.v
    bool is_safe1(float A);
    // OneDimAccShim2.v
    bool is_safe2(float A);

    bool bound_is_safe(float H, float V, float t1,
                       float t2, float a1, float a2);

    // converts the motor signals to acceleration
    float get_acceleration(int16_t motor_out[]);

    // returns the most recently read altitude
    float get_altitude();

    // returns the most recently read vertical velocity
    float get_vertical_vel();

    // the previous acceleration
    float _a;

    // The delay between the sensor readings and when
    // the new motor accelerations will be set.
    // The loop that reads sensors and writes motors
    // is called fast_loop in ArduCopter.pde. It should
    // run at 100hz for SITL simulation. The frequency is
    // determined by the macro MAIN_LOOP_RATE in config.h
    // in ArduCopter.
    const float _d;

    // The altitude upper bound
    const float _ub;

    // the object that gives sensor readings
    const AP_InertialNav_NavEKF& _inertial_nav;

};
