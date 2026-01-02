#ifndef FSI6_CONSTANTS_H
#define FSI6_CONSTANTS_H

const uint16_t REMOTE_CALIBRATION_COUNT = 1000;

/*
 for the Fsi6 remote control the remote; the default position for the
 yaw, roll and pitch values is centered.  With a range from 1000 to 2000, the center position is 1500.
 Therefore the margin or range from the center position is +/- 500.  In order to normalize the
 */
const uint16_t ROLL_MARGIN = 500;
const uint16_t PITCH_MARGIN = 500;
const uint16_t YAW_MARGIN = 500;

const uint16_t THROTTLE_RANGE = 1000;
const uint16_t THROTTLE_MAX = 1000;

#endif // FSI6_CONSTANTS_H

