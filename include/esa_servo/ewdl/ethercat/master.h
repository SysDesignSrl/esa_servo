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

#define READY_TO_SWITCH_ON_BIT       0
#define SWITCHED_ON_BIT              1
#define OPERATION_ENABLED_BIT        2
#define FAULT_BIT                    3
#define VOLTAGE_ENABLED_BIT          4
#define QUICK_STOP_BIT               5
#define SWITCH_ON_DISABLED_BIT       6
#define WANRNING_BIT                 7
#define REMOTE_BIT                   9
#define TARGET_REACHED_BIT          10
#define INTERNAL_LIMIT_ACTIVE_BIT   11


namespace esa { namespace ewdl { namespace ethercat {


inline int slave_setup(uint16 slave)
{
  int wkc = 0;

  // SyncManagers assignment
  uint16 sdo_1c12[] = { 0x01, 0x1600 };     // SM2
  uint16 sdo_1c13[] = { 0x01, 0x1A00 };     // SM3
  wkc += writeSDO<uint16>(slave, 0x1c12, 0x00, sdo_1c12);
  wkc += writeSDO<uint16>(slave, 0x1c13, 0x00, sdo_1c13);

  // SyncManagers synchronnization
  uint16 sdo_1c32[] = { 0x20, 0x0002 };
  uint16 sdo_1c33[] = { 0x20, 0x0002 };
  wkc += writeSDO<uint16>(slave, 0x1c32, 0x01, sdo_1c32[1]);
  wkc += writeSDO<uint16>(slave, 0x1c33, 0x01, sdo_1c33[1]);

  return wkc;
}


class Master {
private:

  int wkc = 0;
  int ec_state = EC_STATE_NONE;

  const static size_t MAX_IO_MAP_SIZE = 4096;
  uint8 io_map[MAX_IO_MAP_SIZE];

  std::string ifname;
  std::vector<std::string> slaves;

  bool network_configuration()
  {
    for (int i = 0; i < slaves.size(); i++)
    {
      const uint16 slave_idx = 1 + i;
      if (strcmp(ec_slave[slave_idx].name, slaves[i].c_str()))
      {
        return false;
      }
    }
    return true;
  }


  void ec_sync(uint64 t_ref, uint64 cycletime, int32 *t_off)
  {
    static int32 integral = 0;
    int32 delta = (t_ref - (cycletime / 2)) % cycletime;

    if (delta > (cycletime / 2))
    {
      delta = delta - cycletime;
    }
    if (delta > 0) integral++;
    if (delta < 0) integral--;

    *t_off = -(delta / 100) - (integral / 20);
  }

public:
  unsigned long t_cycle; int t_off = 0;

  esa::ewdl::ethercat::pdo::RxPDO1 rx_pdo[10];
  esa::ewdl::ethercat::pdo::TxPDO1 tx_pdo[10];

  Master() { }

  Master(unsigned long cycletime, const std::string &ifname, const std::vector<std::string> &slaves) :
    t_cycle(cycletime),
    ifname(ifname),
    slaves(slaves) { }


  bool init()
  {
    if (ec_init(ifname.c_str()) > 0)
    {
      ROS_INFO("EtherCAT socket on: %s", ifname.c_str());
    }
    else
    {
      ROS_ERROR("Coludn't initialize EtherCAT Master socket on: %s", ifname.c_str());
      return false;
    }

    if (ec_config_init(FALSE) > 0)
    {
      ROS_INFO("Slaves found and configured: %d", ec_slavecount);
    }
    else
    {
      ROS_ERROR("Coludn't find and configure any slave.");
      return false;
    }

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
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_dcsync0(slave_idx, TRUE, t_cycle, 0);
    }

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
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
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      print_sm(slave_idx, 0);     // SM0
      print_sm(slave_idx, 1);     // SM1
      print_sm(slave_idx, 2);     // SM2 (output)
      print_sm(slave_idx, 3);     // SM3 (input)
      print_fmmu(slave_idx, 0);   // FMMU0
      print_fmmu(slave_idx, 1);   // FMUU1
    }

    // SAFE OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
      clear_alarm(slave_idx);
    }

    return true;
  }


  bool start()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
      rx_pdo[slave_idx].mode_of_operation = 0;
      rx_pdo[slave_idx].target_position = 0;
      rx_pdo[slave_idx].touch_probe_function = 0;
      rx_pdo[slave_idx].physical_outputs = 0x0000;
    }

    update();

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
    }

    // ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    // print_ec_state(0);
    return true;
  }


  int update()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
    }

    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
    }

    ec_sync(ec_DCtime, t_cycle, &t_off);
    return wkc;
  }


  void close()
  {
    ec_close();
  }


  // Fault Reset

  /* When Fault happens, after the condition that caused the error has been
   * resolved, write 80h to object 0x6040 to clear the error code in object
   * 0x603F and object 0x200F. */

  int fault_reset(uint16 slave_idx)
  {
    uint16 control_word = 0x0080;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
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
    return wkc;
  }

  // Following Error Window
  int set_following_error_window(uint16 slave_idx, uint32 following_error_window = 1000)
  {
    wkc += writeSDO<uint32>(slave_idx, FOLLOWING_ERROR_WINDOW_IDX, 0x00, following_error_window);
    return wkc;
  }

  // In Position
  int config_in_position(uint16 slave_idx, uint16 in_position_counts = 20, uint16 in_position_error_range = 10, uint16 in_position_timing = 10)
  {
    wkc += writeSDO<uint16>(slave_idx, IN_POSITION_COUNTS_IDX, 0x00, in_position_counts);
    wkc += writeSDO<uint16>(slave_idx, IN_POSITION_ERROR_RANGE_IDX, 0x00, in_position_error_range);
    wkc += writeSDO<uint16>(slave_idx, IN_POSITION_TIMING_IDX, 0x00, in_position_timing);
    return wkc;
  }

  // QuickStop Deceleration
  int set_quickstop_deceleration(uint16 slave_idx, uint32 quickstop_deceleration)
  {
    wkc += writeSDO<uint32>(slave_idx, QUICKSTOP_DECELERATION_IDX, 0x00, quickstop_deceleration);
    return wkc;
  }

  // Homing
  int config_homing(uint16 slave_idx, int8 homing_method = 0, uint32 homing_speed_to_switch = 0, uint32 homing_speed_to_zero = 0, uint32 homing_acceleration = 0, int32 home_offset = 0, uint8 home_switch = 0x08)
  {
    wkc += writeSDO<int8>(slave_idx, HOMING_METHOD_IDX, 0x00, homing_method);

    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x01, homing_speed_to_switch);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x02, homing_speed_to_zero);
    wkc += writeSDO<uint32>(slave_idx, HOMING_ACCELERATION_IDX, 0x00, homing_acceleration);

    wkc += writeSDO<int32>(slave_idx, HOME_OFFSET_IDX, 0x00, home_offset);
    wkc += writeSDO<uint8>(slave_idx, HOME_SWITCH_IDX, 0x00, home_switch);

    return wkc;
  }

  // Profile
  int config_profile(uint16 slave_idx, uint32 profile_velocity, uint32 profile_acceleration, uint32 profile_deceleration)
  {
    wkc += writeSDO(slave_idx, PROFILE_VELOCITY_IDX, 0x00, profile_velocity);
    wkc += writeSDO(slave_idx, PROFILE_ACCELERATION_IDX, 0x00, profile_acceleration);
    wkc += writeSDO(slave_idx, PROFILE_DECELERATION_IDX, 0x00, profile_deceleration);
    return wkc;
  }


  int set_zero_position()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      uint8 zero_position;

      zero_position = 0x01;
      wkc += writeSDO<uint8>(slave_idx, ZERO_POSITION_IDX, 0x00, zero_position);

      zero_position = 0x00;
      wkc += writeSDO<uint8>(slave_idx, ZERO_POSITION_IDX, 0x00, zero_position);
    }
    return wkc;
  }


  int get_error_code(const uint16 slave_idx, uint16 &error_code)
  {
    wkc += readSDO<uint16>(slave_idx, ERROR_CODE_IDX, 0x00, error_code);
    return wkc;
  }


  int get_alarm_code(const uint16 slave_idx, uint32 &alarm_code)
  {
    wkc += readSDO<uint32>(slave_idx, ALARM_CODE_IDX, 0x00, alarm_code);
    return wkc;
  }


  bool fault_reset()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0080;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if (!((status_word >> FAULT_BIT) & 0x01)) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool ready_to_switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if ((status_word >> READY_TO_SWITCH_ON_BIT) & 0x01) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0007;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if ((status_word >> SWITCHED_ON_BIT) & 0x01) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool switch_off()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFE;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if (!((status_word >> SWITCHED_ON_BIT) & 0x01)) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool enable_operation()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0008;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if ((status_word >> OPERATION_ENABLED_BIT) & 0x01) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool disable_operation()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFF7;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if (!((status_word >> OPERATION_ENABLED_BIT) & 0x01)) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool quick_stop()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFB;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = tx_pdo[slave_idx].status_word;

        if (!((status_word >> QUICK_STOP_BIT) & 0x01)) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool start_homing()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x001F;
    }
    return true;
  }


  bool halt()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0100;
    }
    return true;
  }


  bool set_mode_of_operation(esa::ewdl::ethercat::mode_of_operation_t mode_of_operation)
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].mode_of_operation = mode_of_operation;
    }

    int iter = 0, max_iter = 10;
    int ok;
    do
    {
      usleep(t_cycle/1000);

      ok = 0;
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 mode_of_operation_display = tx_pdo[slave_idx].mode_of_operation_display;

        if (mode_of_operation_display == mode_of_operation) ok++;
      }

      iter++;
    } while (iter < max_iter && ok < ec_slavecount);

    return (iter < max_iter) ? true : false;
  }


  bool get_mode_of_operation_display(esa::ewdl::ethercat::mode_of_operation_t &mode_of_operation_display)
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      mode_of_operation_display = esa::ewdl::ethercat::mode_of_operation_t(tx_pdo[slave_idx].mode_of_operation_display);
    }
    return true;
  }

};

} } }  // namespace
#endif
