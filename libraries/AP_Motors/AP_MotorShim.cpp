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
// then comes to a stop with acceleration amin.
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

// Gives an estimate of an upper bound on acceleration
// using the throttle value.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float AP_MotorShim::get_acc_from_throttle() {
    return (get_throttle_out()*_throttle_to_accel) + gravity;
}

// Gives an estimate of an upper bound on acceleration
// using the motor values.
// Gives a conservative upper bound by assuming the
// quadcopter is perfectly level and thus all
// motors are pointed upwards.
float AP_MotorShim::get_acceleration(int16_t motor_out[]) {
    float accel = 0;
    for (int8_t i=0; i<4; i++) {
        accel += ((motor_out[i] - 1000)/1000.0)*_pwm_accel_scale;
    }
    return accel + gravity;
}

void AP_MotorShim::set_throttle_from_acc(float A) {
    set_throttle((A-gravity)/_throttle_to_accel);
}

// Sets the motors to match the new total acceleration.
// There is more than one way to set motor_out to deliver the new
// acceleration. To keep things as close as possible to the original
// proposed signals, we keep the ratio between different components
// of motor_out the same. In other words, we linearly scale the motors.
void AP_MotorShim::set_motors_from_acc(float A_new, int16_t motor_out[]) {
    float A_old = get_acceleration(motor_out);
    float ratio = (A_new-gravity)/(A_old-gravity);
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = (ratio*(motor_out[i]-1000.0)) + 1000;
        }
    }
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
// shim) for the next _smooth_lookahead iterations.
float AP_MotorShim::smoothing_shim(float A_proposal) {
    float H = get_altitude();
    float V = get_vertical_vel();

    // The following check would never fail with real arithmetic
    // but could fail with floating point arithmetic.
    if (A_proposal-gravity > 0) {
        return compute_safe2(H, V);
    }

    return 0;
}

// The shim implementation from OneDimAccShim2.v.
// This sets the resulting safe acceleration in _a.
void AP_MotorShim::verified_shim2(float A) {
    float H = get_altitude();
    float V = get_vertical_vel();

    // SafeCtrl is implemented as is_safe2.
    // SafeCtrl takes no argument, but is_safe2
    // must take arguments.
    //
    // \/ /\ SafeCtrl
    //    /\ a! = A
    if (is_safe2(A, H, V)) {
        _a = A;
    }
    // \/ a! = amin
    else {
        _rejected_ver_sum++;
        _accel_diff_ver_sum += A - _amin;
        _a = _amin;
    }

}

// The version of the shim run before the motor
// mixing code. This operates on throttles.
void AP_MotorShim::shim_premix() {
    // Run the unverified smoothing shim
    if (_smooth) {
        float A_proposal = get_acc_from_throttle();
        float A_safe = smoothing_shim(A_proposal);
        if (A_safe < A_proposal) {
            _rejected_unver_sum++;
            _accel_diff_unver_sum += A_proposal - A_safe;
            set_throttle_from_acc(A_safe);
        }
    }

    ///////////////////////////////////////////////////
    // BEGIN SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////

    float A = get_acc_from_throttle();
    verified_shim2(A);
    if (_a < A) {
        set_throttle_from_acc(_a);
    }

    ///////////////////////////////////////////////////
    // END SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////
}

// The version of the shim run after the motor
// mixing code. This operates on values sent to the
// motors.
void AP_MotorShim::shim_postmix(int16_t motor_out[]) {
    // Run the unverified smoothing shim
    if (_smooth) {
        float A_proposal = get_acceleration(motor_out);
        float A_safe = smoothing_shim(A_proposal);
        if (A_safe < A_proposal) {
            _rejected_unver_sum++;
            _accel_diff_unver_sum += A_proposal - A_safe;
            set_motors_from_acc(A_safe, motor_out);
        }
    }

    ///////////////////////////////////////////////////
    // BEGIN SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////

    float A = get_acceleration(motor_out);
    verified_shim2(A);
    if (_a < A) {
        set_motors_from_acc(_a, motor_out);
    }

    ///////////////////////////////////////////////////
    // END SPEC IMPLEMENTATION
    ///////////////////////////////////////////////////
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
// There are actually two implementations of the spec.
// One implementation (shim_premix) occurs before the
// motor mixing code is run and operates on the throttle.
// The other (shim_postmix) runs after the motor mixing
// code is run and operates on the values actually sent to
// the motors.
//
// I've put inline comments in the pre and postmix
// functions to show where each part of this spec is
// implemented. The comments are between
// BEGIN SPEC IMPLEMENTATION and
// END SPEC IMPLEMENTATION.

//
// output_armed - sends control signals to the motors
void AP_MotorShim::output_armed()
{
    // For statistics
    _count++;
    _throttle_sum += get_throttle_out();

    if (_shim_on && _before) {
        shim_premix();
    }
    
    // final outputs sent to the motors
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
    // compute the proposed motor signals
    // these proposed motor signals come
    // from the pilot/other controllers
    compute_outputs_armed(motor_out);

    if (_shim_on && !_before) {
        shim_postmix(motor_out);
    }

    for (int8_t i=0; i<4; i++) {
        _pwm_sum += motor_out[i];
    }

    // write the control signals to the motors
    write_outputs(motor_out);
}
