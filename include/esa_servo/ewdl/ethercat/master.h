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
#include "esa_servo/ewdl/ethercat/pdo.h"
#include "esa_servo/ewdl/ethercat/common.h"

#define EWDL_Z1   1


namespace esa { namespace ewdl { namespace ethercat {

int slave_setup(uint16 slave)
{
  int wkc = 0;

  //
  uint16 sdo_1c12[2] = { 0x0001, 0x1600 }; int size_1c12 = sizeof(sdo_1c12);
  uint16 sdo_1c13[2] = { 0x0001, 0x1a00 }; int size_1c13 = sizeof(sdo_1c13);
  wkc += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, size_1c12, sdo_1c12, EC_TIMEOUTRXM);
  wkc += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, size_1c13, sdo_1c13, EC_TIMEOUTRXM);

  //
  uint16 sdo_1c32[2] = { 0x0020, 0x0001 }; int size_1c32 = sizeof(sdo_1c32);
  uint16 sdo_1c33[2] = { 0x0020, 0x0022 }; int size_1c33 = sizeof(sdo_1c33);
  wkc += ec_SDOwrite(slave, 0x1c32, 0x00, TRUE, size_1c32, sdo_1c32, EC_TIMEOUTRXM);
  wkc += ec_SDOwrite(slave, 0x1c33, 0x00, TRUE, size_1c33, sdo_1c33, EC_TIMEOUTRXM);

  return wkc;
}

class Master
{
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

  void print_ec_state(int ec_state)
  {
    switch (ec_state)
    {
      case EC_STATE_NONE:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_NONE");
        break;
      case EC_STATE_BOOT:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_BOOT");
        break;
      case EC_STATE_INIT:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_INIT");
        break;
      case EC_STATE_PRE_OP:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_PRE_OP");
        break;
      case EC_STATE_SAFE_OP:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_SAFE_OP");
        break;
      case EC_STATE_OPERATIONAL:
        ROS_INFO("%s: %s", ec_slave[EWDL_Z1].name, "EC_STATE_OPERATIONAL");
        break;
      //case EC_STATE_ACK:
      //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
      //  break;
      case EC_STATE_PRE_OP + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[EWDL_Z1].name, "EC_STATE_ERROR", "EC_STATE_PRE_OP");
        break;
      case EC_STATE_SAFE_OP + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[EWDL_Z1].name, "EC_STATE_ERROR", "EC_STATE_SAFE_OP");
        break;
      case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
        ROS_ERROR("%s: %s + %s", ec_slave[EWDL_Z1].name, "EC_STATE_ERROR", "EC_STATE_OPERATIONAL");
        break;
    }
  }

public:

  esa::ewdl::ethercat::pdo::RxPDO0 rx_pdo;
  esa::ewdl::ethercat::pdo::TxPDO0 tx_pdo;

  Master() { }

  Master(const std::string &ifname, const std::vector<std::string> &slaves) : ifname(ifname), slaves(slaves) { }


  bool init()
  {
    if (!ec_init(ifname.c_str()) > 0)
    {
      ROS_ERROR("Coludn't initialize SOEM socket: %s", ifname.c_str());
      return false;
    }

    if (!ec_config_init(FALSE) > 0)
    {
      ROS_ERROR("Coludn't find and configure any slave.");
      return false;
    }

    ROS_INFO("Slaves found and configured: %d", ec_slavecount);
    ec_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    print_ec_state(ec_state);

    // SM0
    ROS_DEBUG("SM0 A:%4.4x L:%4d F:%8.8x Type:%d",
      ec_slave[EWDL_Z1].SM[0].StartAddr,
      ec_slave[EWDL_Z1].SM[0].SMlength,
      ec_slave[EWDL_Z1].SM[0].SMflags,
      ec_slave[EWDL_Z1].SMtype[0]);

    // SM1
    ROS_DEBUG("SM1 A:%4.4x L:%4d F:%8.8x Type:%d",
      ec_slave[EWDL_Z1].SM[1].StartAddr,
      ec_slave[EWDL_Z1].SM[1].SMlength,
      ec_slave[EWDL_Z1].SM[1].SMflags,
      ec_slave[EWDL_Z1].SMtype[1]);

    // SM2 (output)
    ROS_DEBUG("SM2 A:%4.4x L:%4d F:%8.8x Type:%d",
      ec_slave[EWDL_Z1].SM[2].StartAddr,
      ec_slave[EWDL_Z1].SM[2].SMlength,
      ec_slave[EWDL_Z1].SM[2].SMflags,
      ec_slave[EWDL_Z1].SMtype[2]);

    // SM3 (input)
    ROS_DEBUG("SM3 A:%4.4x L:%4d F:%8.8x Type:%d",
      ec_slave[EWDL_Z1].SM[3].StartAddr,
      ec_slave[EWDL_Z1].SM[3].SMlength,
      ec_slave[EWDL_Z1].SM[3].SMflags,
      ec_slave[EWDL_Z1].SMtype[3]);

    // network configuration
    if (!network_configuration())
    {
      ROS_ERROR("Mismatch of network units!");
      return false;
    }


    // Distributed Clock
    //ec_configdc();
    //ec_dcsync0(EWDL_Z1, TRUE, 4000000U, 0);


    // Pre-Operational -> Safe-Operational
    ec_slave[EWDL_Z1].PO2SOconfig = slave_setup;

    uint8 IOmap[128];
    int usedmem = ec_config_map(&IOmap);
    if (usedmem <= sizeof(IOmap))
    {
      ROS_DEBUG("IOmap size: %d", usedmem);
    }


    ROS_DEBUG("FMMU0 Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d",
      ec_slave[EWDL_Z1].FMMU[0].LogStart,
      ec_slave[EWDL_Z1].FMMU[0].LogLength,
      ec_slave[EWDL_Z1].FMMU[0].LogStartbit,
      ec_slave[EWDL_Z1].FMMU[0].LogEndbit,
      ec_slave[EWDL_Z1].FMMU[0].PhysStart,
      ec_slave[EWDL_Z1].FMMU[0].PhysStartBit,
      ec_slave[EWDL_Z1].FMMU[0].FMMUtype,
      ec_slave[EWDL_Z1].FMMU[0].FMMUactive);

    ROS_DEBUG("FMMU1 Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d",
      ec_slave[EWDL_Z1].FMMU[1].LogStart,
      ec_slave[EWDL_Z1].FMMU[1].LogLength,
      ec_slave[EWDL_Z1].FMMU[1].LogStartbit,
      ec_slave[EWDL_Z1].FMMU[1].LogEndbit,
      ec_slave[EWDL_Z1].FMMU[1].PhysStart,
      ec_slave[EWDL_Z1].FMMU[1].PhysStartBit,
      ec_slave[EWDL_Z1].FMMU[1].FMMUtype,
      ec_slave[EWDL_Z1].FMMU[1].FMMUactive);


    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(ec_state);


    // Enable Cyclic Synchronous Position Mode
    int8 sdo_6060 = 0x01;
    wkc += ec_SDOwrite(EWDL_Z1, 0x6060, 0x00, FALSE, sizeof(sdo_6060), &sdo_6060, EC_TIMEOUTRXM);
    int8 sdo_6061 = 0x00; int size_6061 = sizeof(sdo_6061);
    wkc += ec_SDOread(EWDL_Z1, 0x6061, 0x00, FALSE, &size_6061, &sdo_6061, EC_TIMEOUTRXM);
    ROS_DEBUG("WKC: %d SDO 0x6061 Modes of Opration: 0x%.2x", wkc, sdo_6061);

/*
    // Set Running Parameters
    int32 sdo_607A = 0;           // Target Position
    wkc += ec_SDOwrite(EWDL_Z1, 0x607A, 0x00, FALSE, sizeof(sdo_607A), &sdo_607A, EC_TIMEOUTRXM);
    uint32 sdo_6081 = 50000;      // Profile Velocity
    wkc += ec_SDOwrite(EWDL_Z1, 0x6081, 0x00, FALSE, sizeof(sdo_6081), &sdo_6081, EC_TIMEOUTRXM);
    uint32 sdo_6083 = 331666;     // Profile Acceleration
    wkc += ec_SDOwrite(EWDL_Z1, 0x6083, 0x00, FALSE, sizeof(sdo_6083), &sdo_6083, EC_TIMEOUTRXM);
    uint32 sdo_6084 = 331666;     // Profile Deceleration
    wkc += ec_SDOwrite(EWDL_Z1, 0x6084, 0x00, FALSE, sizeof(sdo_6084), &sdo_6084, EC_TIMEOUTRXM);


    // Quickstop Option Code
    uint16 sdo_605A = 0x0002;
    wkc += ec_SDOwrite(EWDL_Z1, 0x605A, 0x00, FALSE, sizeof(sdo_605A), &sdo_605A, EC_TIMEOUTRXM);

    // Halt Option Code
    uint16 sdo_605D = 0x0001;
    wkc += ec_SDOwrite(EWDL_Z1, 0x605D, 0x00, FALSE, sizeof(sdo_605D), &sdo_605D, EC_TIMEOUTRXM);

    // Fault Reaction Option Code
    uint16 sdo_605E = 0x0002;
    wkc += ec_SDOwrite(EWDL_Z1, 0x605E, 0x00, FALSE, sizeof(sdo_605E), &sdo_605E, EC_TIMEOUTRXM);

*/
    // Status Code
    uint32 sdo_200B = 0x0000; int size_200B = sizeof(sdo_200B);
    wkc += ec_SDOread(EWDL_Z1, 0x200B, 0x00, FALSE, &size_200B, &sdo_200B, EC_TIMEOUTRXM);
    ROS_DEBUG("WKC: %d SDO 0x200B Status Code: 0x%.4x", wkc, sdo_200B);
    // Clear Alarm
    //uint8 sdo_2006 = 0x01;
    //wkc += ec_SDOwrite(EWDL_Z1, 0x2006, 0x00, FALSE, sizeof(sdo_2006), &sdo_2006, EC_TIMEOUTRXM);


    // Start/Stopping Motion
    uint16 sdo_6040 = 0x0080;
    wkc += ec_SDOwrite(EWDL_Z1, 0x6040, 0x00, FALSE, sizeof(sdo_6040), &sdo_6040, EC_TIMEOUTRXM);
    uint16 sdo_6041 = 0x0000; int size_6041 = sizeof(sdo_6041);
    wkc += ec_SDOread(EWDL_Z1, 0x6041, 0x00, FALSE, &size_6041, &sdo_6041, EC_TIMEOUTRXM);
    ROS_DEBUG("WKC: %d SDO 0x6041 Status Word: 0x%.4x", wkc, sdo_6041);

    sdo_6040 = 0x0006;
    wkc += ec_SDOwrite(EWDL_Z1, 0x6040, 0x00, FALSE, sizeof(sdo_6040), &sdo_6040, EC_TIMEOUTRXM);
    sdo_6041 = 0x0000; size_6041 = sizeof(sdo_6041);
    wkc += ec_SDOread(EWDL_Z1, 0x6041, 0x00, FALSE, &size_6041, &sdo_6041, EC_TIMEOUTRXM);
    ROS_DEBUG("WKC: %d SDO 0x6041 Status Word: 0x%.4x", wkc, sdo_6041);

    //sdo_6040 = 0x000F;
    //wkc += ec_SDOwrite(EWDL_Z1, 0x6040, 0x00, FALSE, sizeof(sdo_6040), &sdo_6040, EC_TIMEOUTRXM);
    //sdo_6041 = 0x0000; size_6041 = sizeof(sdo_6041);
    //wkc += ec_SDOread(EWDL_Z1, 0x6041, 0x00, FALSE, &size_6041, &sdo_6041, EC_TIMEOUTRXM);
    //ROS_DEBUG("WKC: %d SDO 0x6041 Status Word: 0x%.4x", wkc, sdo_6041);


    // Error Code
    uint16 sdo_603F = 0x0000; int size_603F = sizeof(sdo_603F);
    wkc += ec_SDOread(EWDL_Z1, 0x603F, 0x00, FALSE, &size_603F, &sdo_603F, EC_TIMEOUTRXM);
    ROS_DEBUG("WKC: %d SDO 0x603F Error Code: 0x%.4x", wkc, sdo_603F);

    return true;
  }


  bool start()
  {
    rx_pdo.control_word = 0x0006;
    rx_pdo.mode_of_operation = 1;
    rx_pdo.target_position = 0;
    rx_pdo.touch_probe_function = 0;
    rx_pdo.physical_outputs = 0x0000;

    update();

    // Safe-Operational -> Operational
    ec_slave[EWDL_Z1].state = EC_STATE_OPERATIONAL + EC_STATE_ACK;
    ec_writestate(EWDL_Z1);
    ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    print_ec_state(ec_state);

    if (ec_state != EC_STATE_OPERATIONAL)
    {
      return false;
    }

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
  }

};

} } }  // namespace
#endif
