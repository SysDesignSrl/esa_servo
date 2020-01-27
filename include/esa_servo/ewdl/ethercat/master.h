#ifndef ESA_EWDL_ETHERCAT_MASTER_H
#define ESA_EWDL_ETHERCAT_MASTER_H
// STL
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// SOEM
#include "ethercat.h"
// esa_servo
#include "esa_servo/ewdl/ethercat/registry.h"
#include "esa_servo/ewdl/ethercat/pdo.h"
#include "esa_servo/ewdl/ethercat/common.h"

#define EWDL_Z1   1


namespace esa { namespace ewdl { namespace ethercat {


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, const T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

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


int slave_setup(uint16 slave)
{
  int wkc = 0;

  // PDO mapping
  uint16 sdo_1c12[2] = { 0x0001, 0x1601 }; int size_1c12 = sizeof(sdo_1c12);
  uint16 sdo_1c13[2] = { 0x0001, 0x1a01 }; int size_1c13 = sizeof(sdo_1c13);
  wkc += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, size_1c12, sdo_1c12, EC_TIMEOUTRXM);
  wkc += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, size_1c13, sdo_1c13, EC_TIMEOUTRXM);

  // Sync Managers
  uint16 sdo_1c32[2] = { 0x0020, 0x0002 }; int size_1c32 = sizeof(sdo_1c32);
  uint16 sdo_1c33[2] = { 0x0020, 0x0002 }; int size_1c33 = sizeof(sdo_1c33);
  wkc += ec_SDOwrite(slave, 0x1c32, 0x00, TRUE, size_1c32, sdo_1c32, EC_TIMEOUTRXM);
  wkc += ec_SDOwrite(slave, 0x1c33, 0x00, TRUE, size_1c33, sdo_1c33, EC_TIMEOUTRXM);

  return wkc;
}


class Master {
private:
  std::string ifname;
  std::vector<std::string> slaves;

  int wkc = 0;
  int ec_state = EC_STATE_NONE;


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


  void print_ec_state(uint16 slave, int ec_state)
  {
    switch (ec_state)
    {
      case EC_STATE_NONE:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_NONE");
        break;
      case EC_STATE_BOOT:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_BOOT");
        break;
      case EC_STATE_INIT:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_INIT");
        break;
      case EC_STATE_PRE_OP:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_PRE_OP");
        break;
      case EC_STATE_SAFE_OP:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_SAFE_OP");
        break;
      case EC_STATE_OPERATIONAL:
        ROS_INFO("%s: %s", ec_slave[slave].name, "EC_STATE_OPERATIONAL");
        break;
      //case EC_STATE_ACK:
      //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
      //  break;
      case EC_STATE_PRE_OP + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[slave].name, "EC_STATE_ERROR", "EC_STATE_PRE_OP");
        break;
      case EC_STATE_SAFE_OP + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[slave].name, "EC_STATE_ERROR", "EC_STATE_SAFE_OP");
        break;
      case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[slave].name, "EC_STATE_ERROR", "EC_STATE_OPERATIONAL");
        break;
    }
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
    print_ec_state(0, ec_state);

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

    uint8 IOmap[128];
    int usedmem = ec_config_map(&IOmap);
    if (usedmem <= sizeof(IOmap))
    {
      ROS_DEBUG("IOmap size: %d", usedmem);
    }


    // print slaves configuration
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
      print_sm(slave, 0);     // SM0
      print_sm(slave, 1);     // SM1
      print_sm(slave, 2);     // SM2 (output)
      print_sm(slave, 3);     // SM3 (input)
      print_fmmu(slave, 0);   // FMMU0
      print_fmmu(slave, 1);   // FMUU1
    }

    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0, ec_state);


    // Enable Cyclic Synchronous Velocity Mode
    int8 mode_of_operation = 9;
    wkc += writeSDO<int8>(1, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);

    int8 mode_of_operation_display;
    wkc += readSDO<int8>(1, MODE_OF_OPERATION_DISPLAY_IDX, 0x00, mode_of_operation_display);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Modes of Operation: 0x%.2x", wkc, MODE_OF_OPERATION_IDX, mode_of_operation_display);


    // // Set Running Parameters
    // int32 sdo_607A = 0;           // Target Position
    // wkc += ec_SDOwrite(EWDL_Z1, 0x607A, 0x00, FALSE, sizeof(sdo_607A), &sdo_607A, EC_TIMEOUTRXM);
    // uint32 sdo_6081 = 50000;      // Profile Velocity
    // wkc += ec_SDOwrite(EWDL_Z1, 0x6081, 0x00, FALSE, sizeof(sdo_6081), &sdo_6081, EC_TIMEOUTRXM);
    // uint32 sdo_6083 = 331666;     // Profile Acceleration
    // wkc += ec_SDOwrite(EWDL_Z1, 0x6083, 0x00, FALSE, sizeof(sdo_6083), &sdo_6083, EC_TIMEOUTRXM);
    // uint32 sdo_6084 = 331666;     // Profile Deceleration
    // wkc += ec_SDOwrite(EWDL_Z1, 0x6084, 0x00, FALSE, sizeof(sdo_6084), &sdo_6084, EC_TIMEOUTRXM);
    //
    //
    // // Quickstop Option Code
    // uint16 sdo_605A = 0x0002;
    // wkc += ec_SDOwrite(EWDL_Z1, 0x605A, 0x00, FALSE, sizeof(sdo_605A), &sdo_605A, EC_TIMEOUTRXM);
    //
    // // Halt Option Code
    // uint16 sdo_605D = 0x0001;
    // wkc += ec_SDOwrite(EWDL_Z1, 0x605D, 0x00, FALSE, sizeof(sdo_605D), &sdo_605D, EC_TIMEOUTRXM);
    //
    // // Fault Reaction Option Code
    // uint16 sdo_605E = 0x0002;
    // wkc += ec_SDOwrite(EWDL_Z1, 0x605E, 0x00, FALSE, sizeof(sdo_605E), &sdo_605E, EC_TIMEOUTRXM);


    // Clear Alarm
    //uint8 sdo_2006 = 0x01;
    //wkc += ec_SDOwrite(EWDL_Z1, 0x2006, 0x00, FALSE, sizeof(sdo_2006), &sdo_2006, EC_TIMEOUTRXM);


    // Starting/Stopping Motion
    uint16 control_word = 0x0080;
    wkc += writeSDO<uint16>(1, CONTROL_WORD_IDX, 0x00, control_word);

    uint16 status_word;
    wkc += readSDO<uint16>(1, STATUS_WORD_IDX, 0x00, status_word);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Word: 0x%.4x", wkc, STATUS_WORD_IDX, status_word);


    control_word = 0x0006;
    wkc += writeSDO<uint16>(1, CONTROL_WORD_IDX, 0x00, control_word);

    wkc += readSDO<uint16>(1, STATUS_WORD_IDX, 0x00, status_word);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Word: 0x%.4x", wkc, STATUS_WORD_IDX, status_word);


    // Error Code
    uint16 error_code;
    wkc += readSDO<uint16>(1, ERROR_CODE_IDX, 0x00, error_code);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Error Code: 0x%.4x", wkc, ERROR_CODE_IDX, error_code);


    // Status Code
    uint32 status_code;
    wkc += readSDO<uint32>(1, STATUS_CODE_IDX, 0x00, status_code);
    ROS_DEBUG("WKC: %d SDO 0x%.4x Status Code: 0x%.4x", wkc, STATUS_CODE_IDX, status_code);

    return true;
  }


  bool start_homing()
  {

  }


  bool start()
  {
    rx_pdo.control_word = 0x010F;
    rx_pdo.mode_of_operation = 9;
    rx_pdo.target_velocity = 0;
    rx_pdo.touch_probe_function = 0;
    rx_pdo.physical_outputs = 0x0000;

    update();

    // Safe-Operational -> Operational
    ec_slave[EWDL_Z1].state = EC_STATE_OPERATIONAL + EC_STATE_ACK;
    ec_writestate(EWDL_Z1);
    // ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    // print_ec_state(0, ec_state);

    // if (ec_state != EC_STATE_OPERATIONAL)
    // {
    //   return false;
    // }

    return true;
  }


  int update()
  {
    int wkc = 0;

    rx_pdo >> ec_slave[EWDL_Z1].outputs;
    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET3);
    tx_pdo << ec_slave[EWDL_Z1].inputs;

    return wkc;
  }


  void close()
  {
    ec_close();
    ROS_INFO("EtherCAT socket closed.");
  }

};

} } }  // namespace
#endif
