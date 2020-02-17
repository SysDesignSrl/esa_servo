#ifndef ESA_EWDL_ETHERCAT_COMMON_H
#define ESA_EWDL_ETHERCAT_COMMON_H
// STL
#include <string>
#include <map>
// roscpp
#include <ros/console.h>
// SOEM
#include "ethercat.h"


namespace esa { namespace ewdl { namespace ethercat {

/* Mode of Operation */
enum mode_of_operation_t : int8
{
  Q_PROGRAM = -1,                    // Q Program Mode (ESA specific)
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  TORQUE_PROFILE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
};
// In SM synchronization mode, PP, PV, TQ, HM and Q modes are supported.
// In DC synchronization mode, CSP, CSV and HM modes are supported.


/* Error Codes */
static std::map<uint16, std::string> error_codes
{
  { 0x7500, "EtherCAT communication error"},
  { 0xFF01, "Over Current"},
  { 0xFF02, "Over Voltage"},
  { 0xFF03, "Over Temperature"},
  { 0xFF04, "Open Motor Winding"},
  { 0xFF05, "Internal Voltage Bad"},
  { 0xFF06, "Position Limit"},
  { 0xFF07, "Bad Encoder"},
  { 0xFF08, "reserved"},
  { 0xFF09, "reserved"},
  { 0xFF0A, "Excess regen"},
  { 0xFF0B, "Safe Torque Off"},
  { 0xFF31, "CW Limit"},
  { 0xFF32, "CCW Limit"},
  { 0xFF33, "CW Limit and CCW Limit"},
  { 0xFF34, "Current Foldback"},
  { 0xFF35, "Move while Disabled"},
  { 0xFF36, "Under Voltage"},
  { 0xFF37, "Blank Q segment"},
  { 0xFF41, "Save Failed"},
  { 0xFFFF, "Other Error"},
};


/* Alarm Codes */
static std::map<uint32, std::string> alarm_codes
{
  { 0x00000001, "Position Limit"},
  { 0x00000002, "CCW Limit"},
  { 0x00000004, "CW Limit"},
  { 0x00000008, "Over Temperature"},
  { 0x00000010, "Internal Voltage Bad"},
  { 0x00000020, "Over Voltage"},
  { 0x00000040, "Under Voltage"},
  { 0x00000080, "Over Current"},
  { 0x00000100, "Open Motor Winding"},
  { 0x00000200, "Bad Encoder"},
  { 0x00000400, "Communication Error"},
  { 0x00000800, "Save Failed"},
  { 0x00001000, "No Move"},
  { 0x00002000, "Current Foldback"},
  { 0x00004000, "Blank Q Segment"},
  { 0x00008000, "NV Memory Double Error"},
  { 0x00010000, "Excess Regen"},
  { 0x00020000, "-"},
  { 0x00040000, "Safe Torque Off"},
  { 0x00080000, "-"},
  { 0x00100000, "-"},
};


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, const T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, TRUE, size_of_data, data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T &value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, FALSE, &size_of_data, &data, EC_TIMEOUTRXM);

  value = data;

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, TRUE, &size_of_data, data, EC_TIMEOUTRXM);

  *value = *data;

  return wkc;
}


inline void print_ec_state(uint16 slave)
{
  switch (ec_slave[slave].state)
  {
    case EC_STATE_NONE:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "NONE");
      break;
    case EC_STATE_INIT:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "INIT");
      break;
    case EC_STATE_PRE_OP:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "PRE_OP");
      break;
    case EC_STATE_BOOT:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "BOOT");
      break;
    case EC_STATE_SAFE_OP:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "SAFE_OP");
      break;
    case EC_STATE_OPERATIONAL:
      ROS_INFO("%s: EC_STATE: %s", ec_slave[slave].name, "OPERATIONAL");
      break;
    //case EC_STATE_ACK:
    //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
    //  break;
    case EC_STATE_PRE_OP + EC_STATE_ERROR:
      ROS_ERROR("%s: EC_STATE: %s + %s", ec_slave[slave].name, "PRE_OP", "ERROR");
      break;
    case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      ROS_ERROR("%s: EC_STATE: %s + %s", ec_slave[slave].name, "SAFE_OP", "ERROR");
      break;
    case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      ROS_ERROR("%s: EC_STATE: %s + %s", ec_slave[slave].name, "OPERATIONAL", "ERROR");
      break;
  }
}


inline void print_operation_mode(uint16 slave, int8 operation_mode)
{
  switch (operation_mode)
  {
    case -1:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "Q Program (ESA specific mode)");
      break;
    case 1:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "PP (Profile Position)");
      break;
    case 3:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "PV (Profile Velocity)");
      break;
    case 4:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "TQ (Torque Profile)");
      break;
    case 6:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "HM (Homing)");
      break;
    case 8:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "CSP (Cyclic Synchronous Position)");
      break;
    case 9:
      ROS_INFO("%s: Mode of Operation: %s", ec_slave[slave].name, "CSV (Cyclic Synchronous Velocity)");
      break;
  }
}


inline void print_status_word(uint16 slave, uint16 status_word)
{
  // Ready to Switch On
  if ((status_word >> 0) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Ready to Swith On");
  }
  // Switched On
  if ((status_word >> 1) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Switched On");
  }
  // Operation Enabled
  if ((status_word >> 2) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Operation Enabled");
  }
  // Fault
  if ((status_word >> 3) & 0x01)
  {
    ROS_FATAL("%s: %s", ec_slave[slave].name, "Fault");
  }
  // Voltage Enabled
  if ((status_word >> 4) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Voltage Enabled");
  }
  // Quick Stop
  if ((status_word >> 5) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Quick Stop");
  }
  // Switch On Disabled
  if ((status_word >> 6) & 0x01)
  {
    ROS_WARN("%s: %s", ec_slave[slave].name, "Switch On Disabled");
  }
  // Warning
  if ((status_word >> 7) & 0x01)
  {
    ROS_WARN("%s: %s", ec_slave[slave].name, "Warning");
  }
  // Remote
  if ((status_word >> 9) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Remote");
  }
  // Target Reached
  if ((status_word >> 10) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Target Reached");
  }
  // Internal Limit Active
  if ((status_word >> 11) & 0x01)
  {
    ROS_WARN("%s: %s", ec_slave[slave].name, "Internal Limit Active");
  }
  // Set Point ACK
  if (!(status_word >> 12) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "Previous set point already processed, waiting for new set point");
  }
  else
  {
    ROS_WARN("%s: %s", ec_slave[slave].name, "Previous set point still in process, set point overwriting shall be accepted");
  }
  // Following Error
  if (!(status_word >> 13) & 0x01)
  {
    ROS_INFO("%s: %s", ec_slave[slave].name, "No following error");
  }
  else
  {
    ROS_WARN("%s: %s", ec_slave[slave].name, "Following Error");
  }
}


inline void print_error_code(uint16 slave, uint16 error_code)
{
  ROS_ERROR("%s: Error Code: 0x%.4x %s", ec_slave[slave].name, error_code, error_codes[error_code].c_str());
}


inline void print_alarm_code(uint16 slave, uint32 alarm_code)
{
  ROS_ERROR("%s: Alarm Code: 0x%.8x %s", ec_slave[slave].name, alarm_code, alarm_codes[alarm_code].c_str());
}

} } } // namespace
#endif
