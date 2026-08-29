#pragma once

// Hardware outputs stay locked until the current T870 calibration and an
// explicit bench authorization are recorded.
#ifndef BROON_ENABLE_ACTUATOR_OUTPUTS
#define BROON_ENABLE_ACTUATOR_OUTPUTS 1
#endif

#ifndef BROON_STEERING_CALIBRATION_CONFIRMED
#define BROON_STEERING_CALIBRATION_CONFIRMED 1
#endif

#ifndef BROON_ENABLE_ROS
#define BROON_ENABLE_ROS 0
#endif

#ifndef BROON_ENABLE_HUMAN_SERIAL
#define BROON_ENABLE_HUMAN_SERIAL 1
#endif

// MCP2515 telemetry is monitoring-only. D10-D13 SPI wiring and both bus-end
// termination jumpers were verified before enabling this checked-in default.
#ifndef BROON_ENABLE_CAN_TELEMETRY
#define BROON_ENABLE_CAN_TELEMETRY 1
#endif

// Competition vehicle runs without an occupant. Faults still cut every motor
// output immediately, but they are reported as recoverable DISARMED events so
// the hidden controller does not require a physical RESET to resume.
#ifndef BROON_ENABLE_LATCHED_FAULTS
#define BROON_ENABLE_LATCHED_FAULTS 0
#endif

#if BROON_ENABLE_ROS && BROON_ENABLE_HUMAN_SERIAL
#error "ROS and human-readable Serial cannot share the Uno USB port"
#endif

#if BROON_ENABLE_ACTUATOR_OUTPUTS && !BROON_STEERING_CALIBRATION_CONFIRMED
#error "Actuator outputs require confirmed steering calibration"
#endif
