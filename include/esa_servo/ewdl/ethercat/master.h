#ifndef ESA_EWDL_ETHERCAT_MASTER_H
#define ESA_EWDL_ETHERCAT_MASTER_H
// STL
#include <string>
#include <vector>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// SOEM
#include "ethercat.h"
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
  uint16 sdo_1c12[] = { 0x0001, 0x1601 };     // RxPDO1
  uint16 sdo_1c13[] = { 0x0001, 0x1a01 };     // TxPDO1
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

  esa::ewdl::ethercat::pdo::RxPDO1 rx_pdo;
  esa::ewdl::ethercat::pdo::TxPDO1 tx_pdo;

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
    ec_dcsync0(EWDL_Z1, TRUE, 4000000U, 0);


    // Pre-Operational -> Safe-Operational
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
      ec_slave[slave].PO2SOconfig = slave_setup;
    }

    int used_mem = ec_config_map(&io_map);
    if (used_mem > sizeof(io_map))
    {
      ROS_ERROR("IO Map size: %d > MAX_IO_MAP_SIZE: %lu", used_mem, sizeof(io_map));
      return false;
    }
    ROS_DEBUG("io_map size: %d", used_mem);


    // print slaves configuration
    for (uint16 slave = 1; slave <= ec_slavecount; slave++)
    {
      print_sm(slave, 0);     // SM0
      print_sm(slave, 1);     // SM1
      print_sm(slave, 2);     // SM2 (output)
      print_sm(slave, 3);     // SM3 (input)
      print_fmmu(slave, 0);   // FMMU0
      print_fmmu(slave, 1);   // FMUU1
    }

    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);


    //
    fault_reset();

    //
    init_homing();


    uint16 control_word = 0x0006;
    wkc += writeSDO<uint16>(1, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 status_word;
    wkc += readSDO<uint16>(1, STATUS_WORD_IDX, 0x00, status_word);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Word: 0x%.4x", wkc, STATUS_WORD_IDX, status_word);


    // Status Code
    uint32 status_code;
    wkc += readSDO<uint32>(1, STATUS_CODE_IDX, 0x00, status_code);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Code: 0x%.4x", wkc, STATUS_CODE_IDX, status_code);

    return true;
  }


  // Fault Reset
  int fault_reset()
  {
    uint16 control_word = 0x0080;
    wkc += writeSDO<uint16>(1, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 error_code;
    wkc += readSDO<uint16>(1, ERROR_CODE_IDX, 0x00, error_code);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Error Code: 0x%.4x", wkc, ERROR_CODE_IDX, error_code);

    return wkc;
  }


  // Clear Alarm
  int clear_alarm()
  {
    uint8 clear_alarm = 0x01;
    wkc += writeSDO<uint8>(1, CLEAR_ALARM_IDX, 0x00, clear_alarm);

    uint32 alarm_code;
    wkc += readSDO<uint32>(1, ALARM_CODE_IDX, 0x00, alarm_code);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Alarm Code: 0x%.8x", wkc, ALARM_CODE_IDX, alarm_code);

    return wkc;
  }


  // Homing Mode
  int init_homing()
  {
    int8 homing_method = 2;
    wkc += writeSDO<int8>(1, HOMING_METHOD_IDX, 0x00, homing_method);
    wkc += readSDO<int8>(1, HOMING_METHOD_IDX, 0x00, homing_method);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Homing Method: 0x%.2x", wkc, HOMING_METHOD_IDX, homing_method);

    uint32 homing_speed[] = { 0x02, 10000, 2000 };
    wkc += writeSDO<uint32>(1, HOMING_SPEED_IDX, 0x01, homing_speed[1]);
    wkc += writeSDO<uint32>(1, HOMING_SPEED_IDX, 0x02, homing_speed[2]);
    wkc += readSDO<uint32>(1, HOMING_SPEED_IDX, 0x01, homing_speed[1]);
    wkc += readSDO<uint32>(1, HOMING_SPEED_IDX, 0x02, homing_speed[2]);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Homing Speed: %d %d", wkc, HOMING_SPEED_IDX, homing_speed[1], homing_speed[2]);

    uint32 homing_acceleration = 10000;
    wkc += writeSDO<uint32>(1, HOMING_ACCELERATION_IDX, 0x00, homing_acceleration);

    int32 home_offset = 0;
    wkc += writeSDO<int32>(1, HOME_OFFSET_IDX, 0x00, home_offset);

    uint8 home_switch = 8;
    wkc += writeSDO<uint8>(1, HOME_SWITCH_IDX, 0x00, home_switch);


    // int8 mode_of_operation = 6;
    // wkc += writeSDO<int8>(1, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);
    //
    // int8 mode_of_operation_display;
    // wkc += readSDO<int8>(1, MODE_OF_OPERATION_DISPLAY_IDX, 0x00, mode_of_operation_display);
    // ROS_DEBUG("WKC: %d SDO 0x%.4x Mode of Operation Display: 0x%.2x", wkc, MODE_OF_OPERATION_DISPLAY_IDX, mode_of_operation_display);

    return wkc;
  }


  bool start_homing()
  {
    rx_pdo.control_word = 0x001F;
    rx_pdo.mode_of_operation = 6;
    rx_pdo.target_velocity = 0;
    rx_pdo.touch_probe_function = 0;
    rx_pdo.physical_outputs = 0x0000;

    update();

    // Safe-Operational -> Operational
    ec_slave[1].state = EC_STATE_OPERATIONAL + EC_STATE_ACK;
    ec_writestate(0);

    return true;
  }


  int set_zero_position()
  {
    uint8 zero_position;

    zero_position = 0x01;
    wkc += writeSDO<uint8>(1, ZERO_POSITION_IDX, 0x00, zero_position);

    zero_position = 0x00;
    wkc += writeSDO<uint8>(1, ZERO_POSITION_IDX, 0x00, zero_position);

    return wkc;
  }


  int init_profile()
  {
    int32 target_position = 0;
    wkc += writeSDO(EWDL_Z1, TARGET_POSITION_IDX, 0x00, target_position);

    uint32 profile_velocity = 200000;
    wkc += writeSDO(EWDL_Z1, PROFILE_VELOCITY_IDX, 0x00, profile_velocity);

    uint32 profile_acceleration = 200000;
    wkc += writeSDO(EWDL_Z1, PROFILE_ACCELERATION_IDX, 0x00, profile_acceleration);

    uint32 profile_deceleration = 200000;
    wkc += writeSDO(EWDL_Z1, PROFILE_DECELERATION_IDX, 0x00, profile_deceleration);

    return wkc;
  }


  // Enable Cyclic Synchronous Velocity Mode

  /* In this mode the master controller sends target velocity (0x60FF) to the
   * drive at every PDO update cycle. The primary feedback from the drive is the
   * actual motor position and optionally, actual motor velocity and torque.
   * Velocity and torque control loops are closed in the drive. If necessary,
   * position loop is closed in the master controller. */

  int init_cyclic_syncronous_velocity()
  {
    int8 mode_of_operation = 9;
    wkc += writeSDO<int8>(1, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);

    int8 mode_of_operation_display;
    wkc += readSDO<int8>(1, MODE_OF_OPERATION_DISPLAY_IDX, 0x00, mode_of_operation_display);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Mode of Operation Display: 0x%.2x", wkc, MODE_OF_OPERATION_DISPLAY_IDX, mode_of_operation_display);


    uint16 control_word = 0x0006;
    wkc += writeSDO<uint16>(1, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 status_word;
    wkc += readSDO<uint16>(1, STATUS_WORD_IDX, 0x00, status_word);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Word: 0x%.4x", wkc, STATUS_WORD_IDX, status_word);

    return wkc;
  }


  bool start()
  {
    rx_pdo.control_word = 0x0006;
    rx_pdo.mode_of_operation = 9;
    rx_pdo.target_velocity = 0;
    rx_pdo.touch_probe_function = 0;
    rx_pdo.physical_outputs = 0x0000;

    update();

    // Safe-Operational -> Operational
    ec_slave[1].state = EC_STATE_OPERATIONAL + EC_STATE_ACK;
    ec_writestate(1);

    return true;
  }


  int update()
  {
    rx_pdo >> ec_slave[EWDL_Z1].outputs;
    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET3);
    tx_pdo << ec_slave[EWDL_Z1].inputs;

    return wkc;
  }


  void close()
  {
    ec_close();
  }

};

} } }  // namespace
#endif
