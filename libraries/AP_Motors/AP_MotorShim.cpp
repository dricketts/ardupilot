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
static float sdist(float V, float amin) {
    return -(V*V)/(2*amin);
}

// Gives the height of the vehicle if it starts at H with
// initial velocity V, travels for t1 time at acceleration
// a1, then travels for t2 time at acceleration a2, and
// then comes to a stop with acceleration _amin.
float AP_MotorShim::bound_expr(float H, float V, float t1,
                               float t2, float a1, float a2) {
    return H + tdist(V, a1, t1)
        + tdist(V + (a1*_d), a2, t2)
        + sdist(V + (a1*_d) + (a2*t2), _amin);
}

// Returns true iff bound_expr on the same arguments is at most
// the verified shim's upper bound
bool AP_MotorShim::bound_is_safe(float H, float V, float t1,
                                  float t2, float a1, float a2) {
    return bound_expr(H, V, t1, t2, a1, a2) <= _ub_shim;
}

// safety check on the proposed acceleration
// taken from OneDimAccShim1.v.
// A - the proposed acceleration
bool AP_MotorShim::is_safe1(float A, float H, float V) {
    return bound_is_safe(H, V, _d, _d, _amax, _amax) &&
        bound_is_safe(H, V, 0, _d, _amax, _amax);
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

// The expression appearing in the square root of safe_accel2.
// It's useful to factor this out so that we can check for
// non-negativity in the argument we pass.
float AP_MotorShim::sqrt_expr(float H, float V, float t1,
                              float t2, float a1) {
    return _amin*((4*a1*_d*t2) + (4*a1*t1*t1) + (8*H) +
                 (_amin*t2*t2) + (8*t1*V) + (4*t2*V) - (8*_ub_smooth));

}

// Returns the maximum value for a2 that would return true
// for bound_is_safe(H, V, t1, t2, a1, a2). Returns 0 if
// no such value exists.
float AP_MotorShim::safe_accel2(float H, float V, float t1,
                                float t2, float a1) {
    if (sqrt_expr(H, V, t1, t2, a1) >= 0) {
        return (sqrt(sqrt_expr(H, V, t1, t2, a1))
                - (2*a1*_d) + (_amin*t2) - (2*V))
            /(2*t2);
    } else {
        return 0;
    }
}

// Computes the maximum value A that would allow the
// vehicle to stop in time if we apply acceleration
// A for the next _smooth_lookahead iterations of this
// shim rather than just for the next iteration.
// Returns 0 if no such value is found.
float AP_MotorShim::compute_safe2(float H, float V) {
    float lookahead = _d*_smooth_lookahead;
    if (_a >= 0 && tdist(V, _a, _d) >= 0) {
        return safe_accel2(H, V, _d, lookahead, _a);
    } else if (_a >= 0 && tdist(V, _a, _d) < 0) {
        return safe_accel2(H, V, 0, lookahead, _a);
    } else if (_a < 0 && tdist(V, 0, _d) >= 0) {
        return safe_accel2(H, V, _d, lookahead, 0);
    } else if (_a < 0 && tdist(V, 0, _d) < 0) {
        return safe_accel2(H, V, 0, lookahead, 0);
    }

    return 0;
}

// Gives an estimate of an upper bound on acceleration.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float AP_MotorShim::get_acceleration() {
    return (get_throttle_out()*_throttle_to_accel) + gravity;
}

void AP_MotorShim::set_throttle_from_acc(float A) {
    set_throttle((A-gravity)/_throttle_to_accel);
}

// gets the most recently estimated altitude
float AP_MotorShim::get_altitude() {
    return _inertial_nav.get_altitude();
}

// gets the most recently estimated vertical velocity
float AP_MotorShim::get_vertical_vel() {
    return _inertial_nav.get_velocity_z();
}

// This shim is used to try to smooth the motion of the vehicle,
// rather than engaging the harsh deceleration of the verified
// shim. The shim compute the maximum acceleration that will be
// safe (won't engage the breaking safety mode of the verified
// shim) for the next _smooth_lookahead iterations. If this
// value is less than the acceleration induced by the input
// motor_out, then motor_out is set to deliver this acceleration.
// There is more than one way to set motor_out to deliver the new
// acceleration. To keep things as close as possible to the original
// proposed signals, we keep to ratio between different components
// of motor_out the same.
void AP_MotorShim::smoothing_shim() {
    float H = get_altitude();
    float V = get_vertical_vel();
    float A_proposal = get_acceleration();

    float A_safe = compute_safe2(H, V);
    if (A_safe < A_proposal) {
        set_throttle_from_acc(A_safe);
    }
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
    // Run the unverified smoothing shim
    //smoothing_shim();

    ///////////////////////////////////////////////////
    // BEGIN SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////

    // These are variable mappings. These would need
    // to be specified by the user.

    float H = get_altitude();
    float V = get_vertical_vel();
    float A = get_acceleration();

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
        //   1) A write to the throttle
        //   2) A write to the variable storing the previous
        //      acceleration.
        // We didn't have to do part (1) in the if block
        // because no changes need to be made to motor_out.
        // I'm not sure how to handle these two cases in a
        // uniform way.
        set_throttle_from_acc(_amin);
        _a = _amin;
    }

    ///////////////////////////////////////////////////
    // END SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////
    
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors
    // compute the proposed motor signals
    // these proposed motor signals come
    // from the pilot/other controllers
    compute_outputs_armed(motor_out);

    // write the control signals to the motors
    write_outputs(motor_out);
}
