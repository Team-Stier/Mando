#pragma once

// Hardware outputs stay locked until the current T870 calibration and an
// explicit bench authorization are recorded.
#ifndef BROON_ENABLE_ACTUATOR_OUTPUTS
#define BROON_ENABLE_ACTUATOR_OUTPUTS 1
#endif

#ifndef BROON_ENABLE_DRIVE_OUTPUTS
#define BROON_ENABLE_DRIVE_OUTPUTS 1
#endif

#ifndef BROON_STEERING_CALIBRATION_CONFIRMED
#define BROON_STEERING_CALIBRATION_CONFIRMED 1
#endif

#ifndef BROON_ENABLE_ROS
#define BROON_ENABLE_ROS 1
#endif

#ifndef BROON_ENABLE_HUMAN_SERIAL
#define BROON_ENABLE_HUMAN_SERIAL 0
#endif

#if BROON_ENABLE_ROS && BROON_ENABLE_HUMAN_SERIAL
#error "ROS and human-readable Serial cannot share the Uno USB port"
#endif

#if BROON_ENABLE_ACTUATOR_OUTPUTS && !BROON_STEERING_CALIBRATION_CONFIRMED
#error "Actuator outputs require confirmed steering calibration"
#endif
