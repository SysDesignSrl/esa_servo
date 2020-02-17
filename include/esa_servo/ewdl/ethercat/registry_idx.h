#ifndef ESA_EWDL_ETHERCAT_REGISTRY_IDX_H
#define ESA_EWDL_ETHERCAT_REGISTRY_IDX_H


namespace esa { namespace ewdl { namespace ethercat {

/* This object is used to configure the output sync manager. */
const uint16 SYNC_MANAGER_OUTPUT_PARAMETER_IDX = 0x1C32;

/* This object is used to configure the input sync manager. */
const uint16 SYNC_MANAGER_INPUT_PARAMETER_IDX = 0x1C33;


/* This object reads back the most recent error code generated by the drive. */
const uint16 ERROR_CODE_IDX = 0x603F;

/* This object is used to control the state and motion of the drive.
 * It can be used to enable / disable the driver power output, start, and abort
 * moves in all operating modes, and clear fault conditions. */
const uint16 CONTROL_WORD_IDX = 0x6040;

/* The object indicates the current state of the drive. It consists of bits that
 * indicate the state according to the drive and operation mode.*/
const uint16 STATUS_WORD_IDX = 0x6041;

/* This object is used to set operation mode. */
const uint16 MODE_OF_OPERATION_IDX = 0x6060;
const uint8 MODE_OF_OPERATION_SUB = 0x00;

/* This object displays current operation mode of the drive.
 * Definition of value is same as Mode of Operation (0x6060). */
const uint16 MODE_OF_OPERATION_DISPLAY_IDX = 0x6061;

/* This object provides the actual value of the position measured by encoder.
 * The unit of this object is count. */
const uint16 POSITION_ACTUAL_VALUE_IDX = 0x6064;

/* This object indicates the configured range of tolerated position values
 * symmetrically to the position demand value.
 * If the position actual value is out of the following error window, a following
 * error (Position Limit) occurs.
 * A following error may occur when a drive is blocked, unreachable profile velocity
 * occurs (Jog mode should be Mode 1), or at wrong closed-loop coefficients.
 * The unit of this object is encoder count. If the value of the following error
 * window is 0, the following control shall be switched off. */
const uint16 FOLLOWING_ERROR_WINDOW_IDX = 0x6065;

/* This object provides the actual velocity value derived from encoder.
 * The unit of this object is count/s. */
const uint16 VELOCITY_ACTUAL_VALUE_IDX = 0x606C;

/* This object is the input value for the torque controller in Torque Profile Mode.
 * This object can only be accessed in Servo (or Step Servo) drive.
 * The unit of this object is mN·m. */
const uint16 TARGET_TORQUE_IDX = 0x6071;

/* This object configures the max current permissible value of the drive.
 * The unit of this object is 0.01Amps. */
const uint16 MAX_CURRENT_IDX = 0x6073;

/* This parameter is the output value of the torque limit function (if available
 * within the torque control and power-stage function). This object can only be
 * accessed in Servo (or Step Servo) drive. The unit of this object is mN·m. */
const uint16 TARGET_DEMAND_VALUE_IDX = 0x6074;

/* This object refers to the instantaneous current in the motor. This object can
 * only be accessed in Servo (or Step Servo) drive. The unit of this object is
 * 0.01Amps.*/
const uint16 CURRENT_ACTUAL_VALUE_IDX = 0x6078;

/* This object specifies the target position in Profile Position (PP) mode and
 * Cyclic Synchronous Position (CSP) mode.
 * The unit of this object is count. It is related to encoder resolution.
 * It is used as absolute coordinate or relative coordinate depending on the
 * Bit 6 (0x6040.6) setting of the Control Word in the PP mode, and is always
 * used as absolute value in the CSP mode. */
const uint16 TARGET_POSITION_IDX = 0x607A;

/* This object is the difference between the zero position for the application
 * and the home sensor position (found during homing).
 * During homing the home sensor position is found and once the homing is completed
 * the zero position is offset from the home position by adding the Home Offset to
 * the home position.
 * All subsequent absolute moves shall be taken relative to this new zero position.
 * The unit of this object is count. */
const uint16 HOME_OFFSET_IDX = 0x607C;

/* This object configures the maximum speed allowed in either direction in a move
 * profile. The unit of this object is count/s. */
const uint16 MAX_PROFILE_SPEED_IDX = 0x607F;

/* This object configures the velocity normally attained at the end of the acceleration
 * ramp during a profiled move and is valid for both directions of motion.
 * This object sets the velocity value except the velocity parameter in Profile
 * Velocity mode (pv). The unit of this object is count/s. */
const uint16 PROFILE_VELOCITY_IDX = 0x6081;

/* This object configures the acceleration ramp in a profiled move.
 * The unit of this object is count/s^2 . */
const uint16 PROFILE_ACCELERATION_IDX = 0x6083;

/* This object configures the deceleration ramp in a profiled move.
 * The unit of this object is count/s^2 .*/
const uint16 PROFILE_DECELERATION_IDX = 0x6084;

/* This object configures deceleration used to stop the motor when the quick stop
 * function is activated. The unit of this object is count/s^2 . */
const uint16 QUICKSTOP_DECELERATION_IDX = 0x6084;

/* This object describes the rate of change of torque.
 * The unit of this object is mN·m/s. */
const uint16 TORQUE_SLOPE_IDX = 0x6087;

/* This object determines the method that will be used during homing. */
const uint16 HOMING_METHOD_IDX = 0x6098;

/* This object determines the speeds that will be used during homing.
 * There are two parts to define those speeds.
 * Sub-index 1 to set the speed to search home switch.
 * Sub-index 2 to set the speed to search zero position.
 * The unit of this object is count/s. */
const uint16 HOMING_SPEED_IDX = 0x6099;

/* This object establishes the acceleration to be used for all accelerations and
 * decelerations with the standard homing modes.
 * The unit of this object is count/s 2 . */
const uint16 HOMING_ACCELERATION_IDX = 0x609A;

/* This object configures the function of touch probe. */
const uint16 TOUCH_PROBE_FUNCTION_IDX = 0x60B8;

/* This object provides the status of touch probe. */
const uint16 TOUCH_PROBE_STATUS_IDX = 0x60B9;

/* This object provides the position value triggered by Touch Probe 1 at rising
 * edge. */
const uint16 TOUCH_PROBE_POSITION_1_POSITIVE_VALUE_IDX = 0x60BA;

/* This object provides the position value triggered by Touch Probe 1 at falling
 * edge. */
const uint16 TOUCH_PROBE_POSITION_1_NEGATIVE_VALUE_IDX = 0x60BB;

/* This object provides the position value triggered by Touch Probe 2 at rising
 * edge. */
const uint16 TOUCH_PROBE_POSITION_2_POSITIVE_VALUE_IDX = 0x60BC;

/* This object provides the position value triggered by Touch Probe 2 at falling
 * edge. */
const uint16 TOUCH_PROBE_POSITION_2_NEGATIVE_VALUE_IDX = 0x60BD;

/* This object displays the actual position error (following error) between the
 * target position and the actual position.
 * The unit of this object is encoder count. */
const uint16 FOLLOWING_ERROR_ACTUAL_VALUE_IDX = 0x60F4;

/* This object monitors the inputs status of the drive. */
const uint16 DIGITAL_INPUTS_IDX = 0x60FD;

/* This object configures or monitors the digital outputs of the drive. */
const uint16 DIGITAL_OUTPUTS_IDX = 0x60FE;

/* This object configures the velocity parameters in Profile Velocity Mode and
 * Cyclic Sync Velocity Mode. The unit of this object is count/s. */
const uint16 TARGET_VELOCITY_IDX = 0x60FF;

/* This object provides information on the supported drive modes. */
const uint16 SUPPORTED_DRIVE_MODES_IDX = 0x6502;


/* This object configures which input is used as the home switch in Homing. */
const uint16 HOME_SWITCH_IDX = 0x2001;

/* This object reads back the status of the drive’s outputs. */
const uint16 OUTPUT_STATUS_IDX = 0x2002;

/* This object configures the motor’s torque constant. The unit of this object
 * is mN·m/Amps. */
const uint16 TORQUE_CONSTANT_IDX = 0x2005;

/* This object provides the feature to clear alarm of the drives.
 * After the condition that caused the error has been resolved, set this value
 * to 01h can clear the error code in object 0x603F and object 0x200F */
const uint16 CLEAR_ALARM_IDX = 0x2006;

/* This object represents the current status code of the drive. */
const uint16 STATUS_CODE_IDX = 0x200B;

/* This object provides the feature to clear the position value in 0x6064
 * (Position actual value). Set this value to 01h can clear the position value
 * in 0x6064 (Position actual value). */
const uint16 ZERO_POSITION_IDX = 0x200C;

/* This object reads back a hexadecimal value of the most recent alarm code of
 * the drive. */
const uint16 ALARM_CODE_IDX = 0x200F;

} } }
#endif