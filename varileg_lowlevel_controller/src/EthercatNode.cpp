#include "varileg_lowlevel_controller/EthercatNode.hpp"

namespace varileg_lowlevel_controller {

bool EthercatNode::init()
{
  ethercat_bus_ = std::make_shared<soem_interface::EthercatBusBase>("eth0");

  epos_ethercat_slave_ = std::make_shared<EposEthercatSlave>("epos1", ethercat_bus_, 1);

  ethercat_bus_->addSlave(epos_ethercat_slave_);

  ethercat_bus_->startup();

  ethercat_bus_->setState(EC_STATE_OPERATIONAL);
  ethercat_bus_->waitForState(EC_STATE_OPERATIONAL);

  MELO_INFO("init called");

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

void EthercatNode::cleanup()
{
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");
}

bool EthercatNode::update(const any_worker::WorkerEvent& event)
{
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");
  return true;
}

void EthercatNode::preCleanup()
{
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void EthercatNode::subscriberCallback(const std_msgs::StringConstPtr &msg)
{
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}

}
