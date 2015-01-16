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
bool AP_MotorShim::is_safe1(float A, float H, float V) {
    return bound_is_safe(H, V, _d, _d, amax, amax) &&
        bound_is_safe(H, V, 0, _d, amax, amax);
}

// safety check on the proposed acceleration
// taken from OneDimAccShim2.v.
// A - the proposed acceleration
bool AP_MotorShim::is_safe2(float A, float H, float V) {
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

// This function is, in a loose sense, an implementation
// of Ctrl from OneDimAccShim1.v or OneDimAccShim2.v,
// depending on which version of is_safe you use. Let's
// consider OneDimAccShim2.v:
//
//   Ctrl ==
//     \/ /\ SafeCtrl
//        /\ a! = A
//     \/ a! = amin
//
// I've put inline comments below to show where each
// part of this spec is implemented. The comments
// are between BEGIN SPEC IMPLEMENTATION and
// END SPEC IMPLEMENTATION.
//
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

    ///////////////////////////////////////////////////
    // BEGIN SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////

    // These are variable mappings. These would need
    // to be specified by the user.
    float A = get_acceleration(motor_out);
    float H = get_altitude();
    float V = get_vertical_vel();

    // SafeCtrl is implemented as is_safe2.
    // SafeCtrl takes no argument, but is_safe2
    // must take arguments whose value is the expressions
    // specified by the variable mappings. We could just
    // put all of these expressions inline, but then
    // get_acceleration would be recomputed many times.
    // Performance is important here.
    //
    // \/ /\ SafeCtrl
    //    /\ a! = A
    if (is_safe2(A, H, V)) {
        _a = A;
    }
    // \/ a! = amin
    else {
        // The implementation of a! = amin is a bit weird.
        // It actually corresponds to two C++ statements:
        //   1) A write to the motor control signal array
        //      (motor_out).
        //   2) A write to the variable storing the previous
        //      acceleration.
        // We didn't have to do part (1) in the if block
        // because no changes need to be made to motor_out.
        // I'm not sure how to handle these two cases in a
        // uniform way.
        for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
        _a = amin;
    }


    ///////////////////////////////////////////////////
    // END SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////

    // write the control signals to the motors
    write_outputs(motor_out);
}
