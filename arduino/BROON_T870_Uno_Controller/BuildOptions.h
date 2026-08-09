#pragma once

// Hardware outputs stay locked until the current T870 calibration and an
// explicit bench authorization are recorded.
#ifndef BROON_ENABLE_ACTUATOR_OUTPUTS
#define BROON_ENABLE_ACTUATOR_OUTPUTS 0
#endif

#ifndef BROON_STEERING_CALIBRATION_CONFIRMED
#define BROON_STEERING_CALIBRATION_CONFIRMED 0
#endif

#ifndef BROON_ESTOP_POLARITY_CONFIRMED
#define BROON_ESTOP_POLARITY_CONFIRMED 0
#endif

#ifndef BROON_ENABLE_ROS
#define BROON_ENABLE_ROS 0
#endif

#ifndef BROON_ENABLE_HUMAN_SERIAL
#define BROON_ENABLE_HUMAN_SERIAL 1
#endif

#if BROON_ENABLE_ROS && BROON_ENABLE_HUMAN_SERIAL
#error "ROS and human-readable Serial cannot share the Uno USB port"
#endif

#if BROON_ENABLE_ACTUATOR_OUTPUTS && !BROON_STEERING_CALIBRATION_CONFIRMED
#error "Actuator outputs require confirmed steering calibration"
#endif

#if BROON_ENABLE_ACTUATOR_OUTPUTS && !BROON_ESTOP_POLARITY_CONFIRMED
#error "Actuator outputs require confirmed E-stop polarity"
#endif
