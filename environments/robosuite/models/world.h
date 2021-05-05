//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_WORLD_H
#define FASTDRL_WORLD_H

#include "base.h"
#include "../utils/macros.h"

// get asset root on start up

class World : public MujocoXML
{

public:
    World(): MujocoXML(mjcfUtils::xmlPathCompleted("base.xml"))
    {
        auto options = mjcfUtils::find_elements(this->root, "option", std::nullopt, true);
        options.set("timestep", mjcfUtils::convertToString(SIMULATION_TIMESTEP));
    }

};


#endif //FASTDRL_WORLD_H

