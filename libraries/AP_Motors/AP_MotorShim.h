// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorShim.h
/// @brief	Motor shim class for Quadcopters

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include "AP_MotorsQuad.h"    // Parent Motors Quad library

/// @class      AP_MotorShim
class AP_MotorShim : public AP_MotorsQuad {
public:

    /// Constructor
    AP_MotorShim( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw,
                  const AP_InertialNav_NavEKF& nav, AP_Baro& baro, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT)
        : AP_MotorsQuad(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz), inertial_nav(nav), a(0) {
    };

protected:
    // output - sends commands to the motors
    virtual void        output_armed();

private:
    // OneDimAccShim1.v
    bool is_safe1(float A);
    // OneDimAccShim2.v
    bool is_safe2(float A);

    // converts the motor signals to acceleration
    float get_acceleration(int16_t motor_out[]);

    // returns the most recently read altitude
    float get_altitude();

    // returns the most recently read vertical velocity
    float get_vertical_vel();

    // the previous acceleration
    float a;

    // the object that gives sensor readings
    const AP_InertialNav_NavEKF& inertial_nav;

};
