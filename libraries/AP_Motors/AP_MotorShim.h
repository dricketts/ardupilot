// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorShim.h
/// @brief	Motor shim class for Quadcopters

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include "AP_MotorsQuad.h"    // Parent Motors Quad library
#include "AP_Motors_Class.h"    // Parent Motors Quad library

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
// Tools/autotest/pysim/multicopter.py
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
  static const float pwm_accel_scale = 544.8138888889;
#endif
// Acceleration due to gravity in cm/s/s.
// Sensors seem to use cm as length unit.
// This constant has same level of precision
// as python simulation.
static const float gravity = -980.665;
// Maximum possible acceleration.
static const float amax = (4 * pwm_accel_scale) + gravity;
// Minimum acceleration for the shim
static const float amin = 500 + gravity;

/// @class      AP_MotorShim
class AP_MotorShim : public AP_MotorsQuad {
public:

    /// Constructor
    // In addition to the arguments required by the parent class,
    // this constructor takes an altitude upper bound and a delay d
    // which gives the maximum time between updates of the shim
    // using new altitude and vertical velocity estimates.
    AP_MotorShim( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw,
                  const AP_InertialNav_NavEKF& nav, const float ub, const float smooth_lookahead,
                  const float d, const int16_t mid_throttle, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT)
        : AP_MotorsQuad(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz), _inertial_nav(nav),
          _ub_shim(ub), _ub_smooth(ub - 1000), _smooth_lookahead(smooth_lookahead), _d(d), _a(0), _shim_on(true) {

    };

    virtual void set_mid_throttle(uint16_t mid_throttle) {
        AP_Motors::set_mid_throttle(mid_throttle);
        _throttle_to_accel = -gravity/mid_throttle;
    }

    void enable_shim() {_shim_on = true;}

    void disable_shim() {_shim_on = false;}

    bool shim_on() {return _shim_on;}

protected:
    // output - sends commands to the motors
    virtual void        output_armed();

private:
    // OneDimAccShim1.v
    bool is_safe1(float A, float H, float V);
    // OneDimAccShim2.v
    bool is_safe2(float A, float H, float V);

    // Gives the height of the vehicle if it starts at H with
    // initial velocity V, travels for t1 time at acceleration
    // a1, then travels for t2 time at acceleration a2, and
    // then comes to a stop with acceleration amin.
    float bound_expr(float H, float V, float t1,
                     float t2, float a1, float a2);

    // Returns true iff bound_expr on the same arguments is at most
    // the verified shim's upper bound
    bool bound_is_safe(float H, float V, float t1,
                       float t2, float a1, float a2);

    // The expression appearing in the square root of safe_accel2.
    // It's useful to factor this out so that we can check for
    // non-negativity in the argument we pass.
    float sqrt_expr(float H, float V, float t1,
                    float t2, float a1);

    // Returns the maximum value for a2 that would return true
    // for bound_is_safe(H, V, t1, t2, a1, a2). Returns 0 if
    // no such value exists.
    float safe_accel2(float H, float V, float t1,
                      float t2, float a1);

    // Computes the maximum value A that would allow the
    // vehicle to stop in time if we apply acceleration
    // A for the next _smooth_lookahead iterations of this
    // shim rather than just for the next iteration.
    // Returns 0 if no such value is found.
    float compute_safe2(float H, float V);

    // This shim is used to try to smooth the motion of the vehicle,
    // rather than engaging the harsh deceleration of the verified
    // shim. The shim compute the maximum acceleration that will be
    // safe (won't engage the breaking safety mode of the verified
    // shim) for the next _smooth_lookahead iterations.
    float smoothing_shim(float A_proposal);

    // converts the motor signals to acceleration
    float get_acc_from_throttle();

    // Gives an estimate of an upper bound on acceleration
    // using the motor values.
    // Gives a conservative upper bound by assuming the
    // quadcopter is perfectly level and thus all
    // motors are pointed upwards.
    float get_acceleration(int16_t motor_out[]);

    void set_throttle_from_acc(float A);

    // Sets the motors to match the new total acceleration.
    // There is more than one way to set motor_out to deliver the new
    // acceleration. To keep things as close as possible to the original
    // proposed signals, we keep the ratio between different components
    // of motor_out the same. In other words, we linearly scale the motors.
    void set_motors_from_acc(float A_new, int16_t motor_out[]);

    // returns the most recently read altitude
    float get_altitude();

    // returns the most recently read vertical velocity
    float get_vertical_vel();

    // The shim implementation from OneDimAccShim2.v.
    // This sets the resulting safe acceleration in _a.
    void verified_shim2(float A);

    // The version of the shim run before the motor
    // mixing code. This operates on throttles.
    void shim_premix();

    // The version of the shim run after the motor
    // mixing code. This operates on values sent to the
    // motors.
    void shim_postmix(int16_t motor_out[]);

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

    // The altitude upper bound for the verified shim
    const float _ub_shim;

    // The altitude upper bound for the smoother
    const float _ub_smooth;

    // The number of iterations that the smoothing shim
    // looks into the future when computing the largest
    // possible safe acceleration
    const float _smooth_lookahead;

    // Ratio between the throttle and the acceleration
    // it produces. Equal to (g/hover_throttle).
    float _throttle_to_accel;

    // the object that gives sensor readings
    const AP_InertialNav_NavEKF& _inertial_nav;

    // flag that is true iff the shim is one
    bool _shim_on;

};
