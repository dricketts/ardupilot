// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_MotorShim.h"

extern const AP_HAL::HAL& hal;

// distance traveled with initial velocity V, acceleration a, for time t
static float tdist(float V, float a, float t) {
    return (V*t) + ((a*t*t)/2);
}

// distance required to stop when traveling
// at positive velocity V
static float sdist(float V) {
    return -(V*V)/(2*amin);
}

bool AP_MotorShim::bound_is_safe(float H, float V, float t1,
                                  float t2, float a1, float a2) {
    return H + tdist(V, a1, t1)
        + tdist(V + (a1*_d), a2, t2)
        + sdist(V + (a1*_d) + (a2*t2)) <= _ub;
}

// safety check on the proposed acceleration
// taken from OneDimAccShim1.v.
// A - the proposed acceleration
bool AP_MotorShim::is_safe1(float A) {
    float H = get_altitude();
    float V = get_vertical_vel();
    return bound_is_safe(H, V, _d, _d, amax, amax) &&
        bound_is_safe(H, V, 0, _d, amax, amax);
}

// safety check on the proposed acceleration
// taken from OneDimAccShim2.v.
// A - the proposed acceleration
bool AP_MotorShim::is_safe2(float A) {
    float H = get_altitude();
    float V = get_vertical_vel();
    return
        (_a >= 0 && tdist(V, _a, _d) >= 0 && A >= 0 && bound_is_safe(H, V, _d, _d, _a, A)) ||
        (_a >= 0 && tdist(V, _a, _d) < 0 && A >= 0 && bound_is_safe(H, V, 0, _d, _a, A)) ||
        (_a < 0 && tdist(V, 0, _d) >= 0 && A >= 0 && bound_is_safe(H, V, _d, _d, 0, A)) ||
        (_a < 0 && tdist(V, 0, _d) < 0 && A >= 0 && bound_is_safe(H, V, 0, _d, 0, A)) ||
        (_a >= 0 && tdist(V, _a, _d) >= 0 && A < 0 && bound_is_safe(H, V, _d, _d, _a, 0)) ||
        (_a >= 0 && tdist(V, _a, _d) < 0 && A < 0 && bound_is_safe(H, V, 0, _d, _a, 0)) ||
        (_a < 0 && tdist(V, 0, _d) >= 0 && A < 0 && bound_is_safe(H, V, _d, _d, 0, 0)) ||
        (_a < 0 && tdist(V, 0, _d) < 0 && A < 0 && bound_is_safe(H, V, 0, _d, 0, 0));
}

// Gives an estimate of an upper bound on acceleration.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float AP_MotorShim::get_acceleration(int16_t motor_out[]) {
    float accel = 0;
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        accel += ((motor_out[i] - 1000)/1000)*pwm_accel_scale;
    }
    return accel;
}

// gets the most recently estimated altitude
float AP_MotorShim::get_altitude() {
    return _inertial_nav.get_altitude();
}

// gets the most recently estimated vertical velocity
float AP_MotorShim::get_vertical_vel() {
    return _inertial_nav.get_velocity_z();
}

// output_armed - sends control signals to the motors
void AP_MotorShim::output_armed()
{
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors
    // compute the proposed motor signals
    // these proposed motor signals come
    // from the pilot/other controllers
    compute_outputs_armed(motor_out);

    // TODO - limit the motor voltages to something that
    // will pass the safety test. This might make flight
    // smoother than repeatedly turning on and off motors.

    float A = get_acceleration(motor_out);

    // turn off the engines if safety condition is not met
    if (!is_safe2(A)) {
        for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
    }

    // store the current acceleration for the next iteration
    _a = A;

    // write the control signals to the motors
    write_outputs(motor_out);
}
