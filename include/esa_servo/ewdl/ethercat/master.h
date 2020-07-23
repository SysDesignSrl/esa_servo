#ifndef ESA_EWDL_ETHERCAT_MASTER_H
#define ESA_EWDL_ETHERCAT_MASTER_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// soem
#include <soem/ethercat.h>
// Boost
#include <boost/variant.hpp>
// esa_servo
#include "esa_servo/ewdl/ethercat/common.h"
#include "esa_servo/ewdl/ethercat/registry_idx.h"
#include "esa_servo/ewdl/ethercat/pdo.h"

#define EWDL_Z1   1


namespace esa { namespace ewdl { namespace ethercat {


inline int slave_setup(uint16 slave)
{
  int wkc = 0;

  // PDO mapping
  uint16 sdo_1c12[] = { 0x0001, 0x1600 };     // RxPDO1
  uint16 sdo_1c13[] = { 0x0001, 0x1a00 };     // TxPDO1
  wkc += writeSDO<uint16>(slave, 0x1c12, 0x00, sdo_1c12);
  wkc += writeSDO<uint16>(slave, 0x1c13, 0x00, sdo_1c13);

  // Sync Managers
  uint16 sdo_1c32[] = { 0x0020, 0x0002 };
  uint16 sdo_1c33[] = { 0x0020, 0x0002 };
  wkc += writeSDO<uint16>(slave, 0x1c32, 0x00, sdo_1c32);
  wkc += writeSDO<uint16>(slave, 0x1c33, 0x00, sdo_1c33);

  return wkc;
}


class Master {
private:
  const static size_t MAX_IO_MAP_SIZE = 4096;

  std::string ifname;
  std::vector<std::string> slaves;

  int wkc = 0;
  int ec_state = EC_STATE_NONE;

  uint8 io_map[MAX_IO_MAP_SIZE];


  bool network_configuration()
  {
    for (int i=0; i < slaves.size(); i++)
    {
      if (strcmp(ec_slave[i+1].name, slaves[i].c_str()))
      {
        return false;
      }
    }
    return true;
  }


  void print_sm(uint16 slave, int sm)
  {
    uint16 A = ec_slave[slave].SM[sm].StartAddr;
    uint16 L = ec_slave[slave].SM[sm].SMlength;
    uint32 F = ec_slave[slave].SM[sm].SMflags;
    uint8 Type = ec_slave[slave].SMtype[sm];

    ROS_DEBUG("SM%d A:%4.4x L:%4d F:%8.8x Type:%d", sm, A, L, F, Type);
  }


  void print_fmmu(uint16 slave, int fmmu)
  {
    uint32 Ls = ec_slave[slave].FMMU[fmmu].LogStart;
    uint16 Ll = ec_slave[slave].FMMU[fmmu].LogLength;
    uint8 Lsb = ec_slave[slave].FMMU[fmmu].LogStartbit;
    uint8 Leb = ec_slave[slave].FMMU[fmmu].LogEndbit;
    uint16 Ps = ec_slave[slave].FMMU[fmmu].PhysStart;
    uint8 Psb = ec_slave[slave].FMMU[fmmu].PhysStartBit;
    uint8 Ty = ec_slave[slave].FMMU[fmmu].FMMUtype;
    uint8 Act = ec_slave[slave].FMMU[fmmu].FMMUactive;

    ROS_DEBUG("FMMU%d Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d", fmmu, Ls, Ll, Lsb, Leb, Ps, Psb, Ty, Act);
  }

public:

  esa::ewdl::ethercat::pdo::RxPDO1 rx_pdo[10];
  esa::ewdl::ethercat::pdo::TxPDO1 tx_pdo[10];

  Master() { }

  Master(const std::string &ifname, const std::vector<std::string> &slaves) : ifname(ifname), slaves(slaves) { }


  bool init()
  {
    if (!ec_init(ifname.c_str()) > 0)
    {
      ROS_FATAL("Coludn't initialize EtherCAT Master socket on: %s", ifname.c_str());
      return false;
    }

    ROS_INFO("EtherCAT socket on: %s", ifname.c_str());


    if (!ec_config_init(FALSE) > 0)
    {
      ROS_FATAL("Coludn't find and configure any slave.");
      return false;
    }

    ROS_INFO("Slaves found and configured: %d", ec_slavecount);


    ec_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    // network configuration
    if (!network_configuration())
    {
      ROS_FATAL("Mismatch of network units!");
      return false;
    }


    // Distributed Clock
    ec_configdc();
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      ec_dcsync0(slave_idx, TRUE, 1000000U, 0);
    }


    // Pre-Operational -> Safe-Operational
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      ec_slave[slave_idx].PO2SOconfig = slave_setup;
    }

    int used_mem = ec_config_map(&io_map);
    if (used_mem > sizeof(io_map))
    {
      ROS_ERROR("IO Map size: %d > MAX_IO_MAP_SIZE: %lu", used_mem, sizeof(io_map));
      return false;
    }
    ROS_DEBUG("io_map size: %d", used_mem);


    // print slaves configuration
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      print_sm(slave_idx, 0);     // SM0
      print_sm(slave_idx, 1);     // SM1
      print_sm(slave_idx, 2);     // SM2 (output)
      print_sm(slave_idx, 3);     // SM3 (input)
      print_fmmu(slave_idx, 0);   // FMMU0
      print_fmmu(slave_idx, 1);   // FMUU1
    }

    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);


    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      fault_reset(slave_idx);
    }

    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      clear_alarm(slave_idx);
    }

    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      init_slave(slave_idx);
    }

    return true;
  }


  // Fault Reset

  /* When Fault happens, after the condition that caused the error has been
   * resolved, write 80h to object 0x6040 to clear the error code in object
   * 0x603F and object 0x200F. */

  int fault_reset(uint16 slave_idx)
  {
    uint16 control_word = 0x0080;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 error_code;
    wkc += readSDO<uint16>(slave_idx, ERROR_CODE_IDX, 0x00, error_code);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Error Code: 0x%.4x", wkc, slave_idx, ERROR_CODE_IDX, error_code);

    return wkc;
  }


  // Clear Alarm

  /* When Warning happens, after the condition that caused the error has been
   * resolved, write 01h to object 0x2006 to clear the error code in object
   * 0x603F and object 0x200F. */

  int clear_alarm(uint16 slave_idx)
  {
    uint8 clear_alarm = 0x01;
    wkc += writeSDO<uint8>(slave_idx, CLEAR_ALARM_IDX, 0x00, clear_alarm);

    uint32 alarm_code;
    wkc += readSDO<uint32>(slave_idx, ALARM_CODE_IDX, 0x00, alarm_code);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Alarm Code: 0x%.8x", wkc, slave_idx, ALARM_CODE_IDX, alarm_code);

    return wkc;
  }


  // Ready to Switch On

  int init_slave(uint16 slave_idx)
  {
    uint16 control_word = 0x0006;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 status_word;
    wkc += readSDO<uint16>(slave_idx, STATUS_WORD_IDX, 0x00, status_word);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Status Word: 0x%.4x", wkc, slave_idx, STATUS_WORD_IDX, status_word);

    // Status Code
    uint32 status_code;
    wkc += readSDO<uint32>(slave_idx, STATUS_CODE_IDX, 0x00, status_code);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Status Code: 0x%.4x", wkc, slave_idx, STATUS_CODE_IDX, status_code);

    // Following Error Window
    uint32 following_error_window = 1000;
    wkc += writeSDO<uint32>(slave_idx, FOLLOWING_ERROR_WINDOW_IDX, 0x00, following_error_window);

    wkc += readSDO<uint32>(slave_idx, FOLLOWING_ERROR_WINDOW_IDX, 0x00, status_code);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Following Error Window: %d", wkc, slave_idx, FOLLOWING_ERROR_WINDOW_IDX, following_error_window);
  }


  // Quick Stop
  int config_quickstop(uint16 slave, uint32 quickstop_deceleration)
  {
    wkc += writeSDO<uint32>(slave, QUICKSTOP_DECELERATION_IDX, 0x00, quickstop_deceleration);
    return wkc;
  }


  // Homing Mode
  int config_homing(uint16 slave_idx, int8 homing_method = 0, uint32 homing_speed_to_switch = 0, uint32 homing_speed_to_zero = 0, uint32 homing_acceleration = 0, int32 home_offset = 0, uint8 home_switch = 0x08)
  {
    wkc += writeSDO<int8>(slave_idx, HOMING_METHOD_IDX, 0x00, homing_method);

    wkc += readSDO<int8>(slave_idx, HOMING_METHOD_IDX, 0x00, homing_method);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Homing Method: %d", wkc, slave_idx, HOMING_METHOD_IDX, homing_method);

    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEED_IDX, 0x01, homing_speed_to_switch);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEED_IDX, 0x02, homing_speed_to_zero);
    wkc += writeSDO<uint32>(slave_idx, HOMING_ACCELERATION_IDX, 0x00, homing_acceleration);

    wkc += readSDO<uint32>(slave_idx, HOMING_SPEED_IDX, 0x01, homing_speed_to_switch);
    wkc += readSDO<uint32>(slave_idx, HOMING_SPEED_IDX, 0x02, homing_speed_to_zero);
    ROS_DEBUG("WKC: %d\tSlave[%u] SDO 0x%.4x Homing Speed: %d %d", wkc, slave_idx, HOMING_SPEED_IDX, homing_speed_to_switch, homing_speed_to_zero);

    wkc += writeSDO<int32>(slave_idx, HOME_OFFSET_IDX, 0x00, home_offset);
    wkc += writeSDO<uint8>(slave_idx, HOME_SWITCH_IDX, 0x00, home_switch);

    return wkc;
  }


  int init_profile(uint16 slave_idx)
  {
    int32 target_position = 0;
    wkc += writeSDO(slave_idx, TARGET_POSITION_IDX, 0x00, target_position);

    uint32 profile_velocity = 200000;
    wkc += writeSDO(slave_idx, PROFILE_VELOCITY_IDX, 0x00, profile_velocity);

    uint32 profile_acceleration = 200000;
    wkc += writeSDO(slave_idx, PROFILE_ACCELERATION_IDX, 0x00, profile_acceleration);

    uint32 profile_deceleration = 200000;
    wkc += writeSDO(slave_idx, PROFILE_DECELERATION_IDX, 0x00, profile_deceleration);

    return wkc;
  }


  bool start()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      fault_reset(slave_idx);
    }

    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      clear_alarm(slave_idx);
    }

    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word = 0x0006;
      rx_pdo[slave_idx].mode_of_operation = mode_of_operation_t::HOMING;
      rx_pdo[slave_idx].target_position = 0;
      rx_pdo[slave_idx].touch_probe_function = 0;
      rx_pdo[slave_idx].physical_outputs = 0x0000;
    }

    update();

    // Safe-Operational -> Operational
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL + EC_STATE_ACK;
      ec_writestate(slave_idx);
    }

    return true;
  }


  bool enable_motion()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word = 0x000F;
      rx_pdo[slave_idx].mode_of_operation = mode_of_operation_t::HOMING;
    }

    return true;
  }


  // Starting the Homing Procedure

  /* Set the Homing Method required using OD entry 6098h. To start the homing
   * procedure, bit 4 of the controlword OD entry located at dictionary address
   * 6040h, must transition from 0 to 1. The status of the homing procedure can
   * be monitored using the statusword OD entry 6041h. */

  bool start_homing()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word = 0x001F;
      rx_pdo[slave_idx].mode_of_operation = mode_of_operation_t::HOMING;
    }

    return true;
  }

  // Enable Cyclic Synchronous Position Mode

  /* In this mode the master controller generates a trajectory and sends target
   * position (0x607A) to the drive at every PDO update cycle. The primary feedback
   * from the drive is the actual motor position and optionally, actual motor
   * velocity and torque. Position, velocity, and torque control loops are all
   closed in the drive which acts as a follower for the position commands. */

  bool start_cyclic_syncronous_position()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word = 0x000F;
      rx_pdo[slave_idx].mode_of_operation =  mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION;
    }

    return true;
  }

  // Enable Cyclic Synchronous Velocity Mode

  /* In this mode the master controller sends target velocity (0x60FF) to the
   * drive at every PDO update cycle. The primary feedback from the drive is the
   * actual motor position and optionally, actual motor velocity and torque.
   * Velocity and torque control loops are closed in the drive. If necessary,
   * position loop is closed in the master controller. */

  bool start_cyclic_syncronous_velocity()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word = 0x000F;
      rx_pdo[slave_idx].mode_of_operation =  mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY;
    }

    return true;
  }


  int update()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
    }

    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET3);

    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
    }

    return wkc;
  }


  // Halt
  bool halt()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word |= 0x0100;
    }

    return true;
  }


  // Quick Stop
  bool quick_stop()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word &= 0b1111111111111011;
    }

    return true;
  }


  void close()
  {
    ec_close();
  }


  int set_zero_position()
  {
    for (uint16 slave = 1; slave <= ec_slavecount; slave++)
    {
      uint8 zero_position;

      zero_position = 0x01;
      wkc += writeSDO<uint8>(slave, ZERO_POSITION_IDX, 0x00, zero_position);

      zero_position = 0x00;
      wkc += writeSDO<uint8>(slave, ZERO_POSITION_IDX, 0x00, zero_position);
    }

    return wkc;
  }


  void print_slave_status(const uint16 slave_idx)
  {
    if ((tx_pdo[slave_idx].status_word >> 0) & 0x01)   // Ready to Switch On
    {
      ROS_INFO("Slave[%d]: Ready to Switch On", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 1) & 0x01)   // Switched On
    {
      ROS_INFO("Slave[%d]: Switched On", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 2) & 0x01)   // Operation Enabled
    {
      ROS_INFO("Slave[%d]: Operation Enabled", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 3) & 0x01)   // Fault
    {
      ROS_ERROR("Slave[%d]: Fault!!", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 4) & 0x01)   // Voltage Enabled
    {
      ROS_INFO("Slave[%d]: Voltage Enabled", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 5) & 0x01)   // Quick Stop
    {
      ROS_INFO("Slave[%d]: Quick Stop Enabled", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 6) & 0x01)   // Switch On Disabled
    {
      ROS_WARN("Slave[%d]: Switch On Disabled", slave_idx);
    }
    if ((tx_pdo[slave_idx].status_word >> 7) & 0x01)   // Warning
    {
      ROS_WARN("Slave[%d]: Warning", slave_idx);
    }

    switch (tx_pdo[slave_idx].mode_of_operation_display)
    {
      case 6:   // HOMING
        if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
        {
          ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
        {
          ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 12) & 0x01)           // Homing Attained
        {
          ROS_INFO("Slave[%d]: Homing Attained.", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Homing Error
        {
          ROS_ERROR("Slave[%d]: Homing Error.", slave_idx);
        }
        break;

      case 8:   // CYCLIC SYNCHRONOUS POSITION
        if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
        {
          ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
        {
          ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
        {
          ROS_ERROR("Slave[%d]: Following Error.", slave_idx);
        }
        break;

      case 9:   // CYCLIC SYNCHRONOUS VELOCITY
        if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
        {
          ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
        {
          ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
        }
        if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
        {
          ROS_ERROR("Slave[%d]: Following Error.", slave_idx);
        }
        break;

      default:

        break;
    }
  }

};

} } }  // namespace
#endif
