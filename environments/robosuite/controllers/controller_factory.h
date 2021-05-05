//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_CONTROLLER_FACTORY_H
#define FASTDRL_CONTROLLER_FACTORY_H

#include <vector>
#include <variant>
#include <string>
#include <unordered_map>
#include "nlohmann/json.hpp"
#include "base_controller.h"

using nlohmann::json;

namespace CtrlFactory {


    json loadControllerConfig(std::string const& custom_fpath="", std::string const& defaultCtrl="");

    std::unique_ptr<BaseController>  controllerFactory(struct MjSim* sim,
                                                       std::string const& name,
                                                       CtrlParameters const& params);


};


#endif //FASTDRL_CONTROLLER_FACTORY_H
