#include "any_node/any_node.hpp"

#include "lowlvl_communication/ExampleNode.hpp"

   
int main(int argc, char **argv) {
    any_node::Nodewrap<lowlvl_communication::ExampleNode> node(argc, argv, "exampleNode", 9); // use 2 spinner threads
    node.execute(); // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
    return 0;
}
