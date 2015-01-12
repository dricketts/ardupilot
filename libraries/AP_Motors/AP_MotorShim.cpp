// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_MotorShim.h"

extern const AP_HAL::HAL& hal;

static int count = 0;

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
    //if (count % 100 == 0) {
    //    hal.console->printf("Altitude: %f\n", inertial_nav.get_altitude());
    //}

    int8_t i;

    if (inertial_nav.get_altitude() >= 5000) {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
    }

    write_outputs(motor_out);
}
