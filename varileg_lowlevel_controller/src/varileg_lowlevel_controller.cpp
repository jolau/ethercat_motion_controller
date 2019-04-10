#include "any_node/any_node.hpp"

#include "varileg_lowlevel_controller/EthercatNode.hpp"

int main(int argc, char **argv) {
  /*if (argc != 2) {
    MELO_ERROR_STREAM("Missing port name (e.g. enp0s31f6).");
    return 0;
  }*/

  any_node::Nodewrap<varileg_lowlevel_controller::EthercatNode>
      node(argc, argv, "ethercatNode", 4); // use 1 spinner threads
  node.execute(); // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
  return 0;
}
