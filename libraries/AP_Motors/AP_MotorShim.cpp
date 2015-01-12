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
static const float d = 0.01;
// The upper bound in cm.
static const float ub = 4000;

//static int count = 0;
//static float height_exceeded = 0;

static float tdist(float V, float a, float t) {
    return (V*t) + ((a*t*t)/2);
}

static float sdist(float V) {
    return -(V*V)/(2*amin);
}

static bool bound_is_safe(float H, float V, float t1, float t2) {
    return H + tdist(V, amax, t1)
        + tdist(V + (amax*d), amax, t2)
        + sdist(V + (amax*d) + (amax*t2)) <= ub;
}

bool AP_MotorShim::is_safe(float accel) {
    float H = inertial_nav.get_altitude();
    float V = inertial_nav.get_velocity_z();
    return bound_is_safe(H, V, d, d) &&
        bound_is_safe(H, V, 0, d);
}

float AP_MotorShim::get_acceleration(int16_t motor_out[]) {
    float accel = 0;
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        accel += ((motor_out[i] - 1000)/1000)*accel_scale;
    }
    return accel;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorShim::output_armed()
{
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors
    compute_outputs_armed(motor_out);

    // Printing stuff out seems to make the
    // quadcopter do crazy things in simulation.
    // This must be a timing issue.
    //count++;
    //if (count % 1000 == 0) {
    //    hal.console->printf("Motor 0: %d\n", motor_out[0]);
    //}

    //    if (inertial_nav.get_altitude() >= ub) {
    //  height_exceeded = inertial_nav.get_altitude();
    //}
    //count++;
    //if (count % 1000 == 0) {
    //  hal.console->printf("Height: %f\n", height_exceeded);
    //}

    int8_t i;

    if (!is_safe(get_acceleration(motor_out))) {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
    }

    write_outputs(motor_out);
}
