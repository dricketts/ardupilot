// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_MotorShim.h"

extern const AP_HAL::HAL& hal;

// Ratio between pwm of a single motor
// (scaled to be between 0 and 1) and the thrust
// it produces. Equal to (g/(4*hover_throttle)).
// Taken from python simulation in
// Tools/autotest/pysim/multicopter.py.
static const float accel_scale = 5.448138888889;
// Maximum possible acceleration.
static const float amax = 4 * accel_scale;
// Acceleration due to gravity in cm/s/s.
// Sensors seem to use cm as length unit.
// This constant has same level of precision
// as python simulation.
static const float amin = -980.665;
// The delay between the sensor readings and when
// the new motor accelerations will be set.
// The loop that reads sensors and writes motors
// is called fast_loop in ArduCopter.pde. It should
// run at 100hz for SITL simulation. The frequency is
// determined by the macro MAIN_LOOP_RATE in config.h
// in ArduCopter.
static const float d = 0.2;
// The upper bound in cm.
static const float ub = 4000;

// distance traveled with initial velocity V, acceleration a, for time t
static float tdist(float V, float a, float t) {
    return (V*t) + ((a*t*t)/2);
}

// distance required to stop when traveling
// at positive velocity V
static float sdist(float V) {
    return -(V*V)/(2*amin);
}

static bool bound_is_safe(float H, float V, float t1, float t2, float a1, float a2) {
    return H + tdist(V, a1, t1)
        + tdist(V + (a1*d), a2, t2)
        + sdist(V + (a1*d) + (a2*t2)) <= ub;
}

// A - the proposed acceleration
bool AP_MotorShim::is_safe1(float A) {
    float H = get_altitude();
    float V = get_vertical_vel();
    return bound_is_safe(H, V, d, d, amax, amax) &&
        bound_is_safe(H, V, 0, d, amax, amax);
}

// A - the proposed acceleration
bool AP_MotorShim::is_safe2(float A) {
    float H = get_altitude();
    float V = get_vertical_vel();
    return
        (a >= 0 && tdist(V, a, d) >= 0 && A >= 0 && bound_is_safe(H, V, d, d, a, A)) ||
        (a >= 0 && tdist(V, a, d) < 0 && A >= 0 && bound_is_safe(H, V, 0, d, a, A)) ||
        (a < 0 && tdist(V, 0, d) >= 0 && A >= 0 && bound_is_safe(H, V, d, d, 0, A)) ||
        (a < 0 && tdist(V, 0, d) < 0 && A >= 0 && bound_is_safe(H, V, 0, d, 0, A)) ||
        (a >= 0 && tdist(V, a, d) >= 0 && A < 0 && bound_is_safe(H, V, d, d, a, 0)) ||
        (a >= 0 && tdist(V, a, d) < 0 && A < 0 && bound_is_safe(H, V, 0, d, a, 0)) ||
        (a < 0 && tdist(V, 0, d) >= 0 && A < 0 && bound_is_safe(H, V, d, d, 0, 0)) ||
        (a < 0 && tdist(V, 0, d) < 0 && A < 0 && bound_is_safe(H, V, 0, d, 0, 0));
}

float AP_MotorShim::get_acceleration(int16_t motor_out[]) {
    float accel = 0;
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        accel += ((motor_out[i] - 1000)/1000)*accel_scale;
    }
    return accel;
}

float AP_MotorShim::get_altitude() {
    return inertial_nav.get_altitude();
}

float AP_MotorShim::get_vertical_vel() {
    return inertial_nav.get_velocity_z();
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorShim::output_armed()
{
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors
    // compute the proposed motor signals
    compute_outputs_armed(motor_out);

    // TODO - limit the motor voltages to something that
    // will pass the safety test. This might make flight
    // smoother than repeatedly turning on and off motors.

    // turn off the engines if safety condition is not met
    if (!is_safe2(get_acceleration(motor_out))) {
        for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
    }

    // store the current acceleration for the next iteration
    a = get_acceleration(motor_out);

    // write the control signals to the motors
    write_outputs(motor_out);
}
