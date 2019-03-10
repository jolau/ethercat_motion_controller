#include "any_node/any_node.hpp"

#include "varileg_lowlevel_controller/EthercatNode.hpp"

   
int main(int argc, char **argv) {
    any_node::Nodewrap<varileg_lowlevel_controller::EthercatNode> node(argc, argv, "ethercatNode", 1); // use 1 spinner threads
    node.execute(); // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
    return 0;
}
