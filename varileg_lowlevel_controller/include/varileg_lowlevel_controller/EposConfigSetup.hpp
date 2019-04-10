//
// Created by jolau on 09.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIGSETUP_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIGSETUP_HPP

// yaml tools
#include "yaml_tools/YamlNode.hpp"

#include <string>
#include <vector>
#include <varileg_lowlevel_controller/entities/EposConfig.hpp>

namespace varileg_lowlevel_controller {
namespace setup {

static std::vector<EposConfig> loadFromFile(const std::string& setupFile) {
  // Load the setup from the file.
  yaml_tools::YamlNode yamlNode = yaml_tools::YamlNode::FromFile(setupFile);

  if(yamlNode.hasKey("joint_config")) {
    const yaml_tools::YamlNode jointConfig = yamlNode["joint_config"];

  }
}

}

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIGSETUP_HPP
