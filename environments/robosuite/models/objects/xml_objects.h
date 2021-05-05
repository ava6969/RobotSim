//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_XML_OBJECTS_H
#define FASTDRL_XML_OBJECTS_H

#include "../../utils/mjcf_utils.h"

class CanObject : public MujocoXMLObject {

public:
    explicit CanObject(string name):MujocoXMLObject(mjcfUtils::xmlPathCompletion("objects/bottle.xml"),
                                                    std::move(name),
                                           { {{"type", "free" }, {"damping", "0,0005"}, {"obj_type", "all"}}},
                                           true)
    {
    }
};


#endif //FASTDRL_XML_OBJECTS_H
