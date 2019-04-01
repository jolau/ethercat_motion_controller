//
// Created by jolau on 28.03.19.
//

#include <varileg_lowlevel_controller/examples/EposExampleNode.hpp>
#include <varileg_lowlevel_controller_msgs/MotorControllerState.h>

bool varileg_lowlevel_controller::examples::EposExampleNode::init() {
  MELO_INFO("init called");

  std::map<std::string, int> joint2eposMap;
  joint2eposMap.insert(std::make_pair("hip_left", 3));
  eposEthercatSlaveManager_->setJointName2NodeIdMap(joint2eposMap);

  eposEthercatSlave_ = std::make_shared<EposEthercatSlave>("epos1", bus_, 1);
  slaves_.push_back(eposEthercatSlave_);

  if(!bus_->startup(slaves_)){
    MELO_ERROR("Startup of bus failed.");
    return false;
  }

  if(!eposEthercatSlaveManager_->addEposEthercatSlave(eposEthercatSlave_)) {
    MELO_ERROR("Could add epos to manager.")
    return false;
  };

  constexpr double defaultWorkerTimeStep = .01;
  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  addWorker("eposExampleNode::updateWorker", workerTimeStep, &EposExampleNode::update, this, priority);

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

void varileg_lowlevel_controller::examples::EposExampleNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void varileg_lowlevel_controller::examples::EposExampleNode::cleanup() {
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");

  bus_->shutdown(slaves_);
}

bool varileg_lowlevel_controller::examples::EposExampleNode::update(const any_worker::WorkerEvent &event) {
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");

  bus_->receiveInbox();

  eposEthercatSlave_->readInbox();

  ExtendedJointState extendedJointState;
  extendedJointState.motorControllerState = MotorControllerState::STATE_OP_ENABLED;
  eposEthercatSlave_->setSendJointState(extendedJointState);

  eposEthercatSlave_->writeOutbox();


 /* varileg_lowlevel_controller_msgs::ExtendedJointStates extendedJointStates = eposEthercatSlaveManager_->updateReadAll();

  for (int i = 0; i < extendedJointStates.name.size(); ++i) {
    if(extendedJointStates.motor_controller_state[i] == varileg_lowlevel_controller_msgs::MotorControllerState::STATE_OP_ENABLED) {
      MELO_INFO_STREAM(extendedJointStates.name[i] << ": Actual Position: " << extendedJointStates.position[i]);
      if (extendedJointStates.position[i] < 5000) {
        extendedJointStates.position[i] = 5100;
      } else {
        extendedJointStates.position[i]= 0;
      }
      MELO_INFO_STREAM(extendedJointStates.name[i] << ": Send Target Position: " << extendedJointStates.position[i]);
    } else {
      MELO_INFO_STREAM(extendedJointStates.name[i] << ": Enabling Drive");
      extendedJointStates.motor_controller_state[i] = varileg_lowlevel_controller_msgs::MotorControllerState::STATE_OP_ENABLED;
    }
  }

  eposEthercatSlaveManager_->updateWriteAll(extendedJointStates);*/

  bus_->sendOutbox();

  return true;
}

void varileg_lowlevel_controller::examples::EposExampleNode::subscriberCallback(const std_msgs::StringConstPtr &msg) {
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}