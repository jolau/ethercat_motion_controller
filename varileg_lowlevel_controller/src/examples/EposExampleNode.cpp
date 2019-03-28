//
// Created by jolau on 28.03.19.
//

#include <varileg_lowlevel_controller/examples/EposExampleNode.hpp>

bool varileg_lowlevel_controller::examples::EposExampleNode::init() {
  MELO_INFO("init called");



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

  bus_->receiveBuffer();

  eposEthercatSlaveManager_->updateWriteAll();

  eposEthercatSlaveManager_->updateReadAll();

  bus_->sendBuffer();

  return true;
}

void varileg_lowlevel_controller::examples::EposExampleNode::subscriberCallback(const std_msgs::StringConstPtr &msg) {
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}