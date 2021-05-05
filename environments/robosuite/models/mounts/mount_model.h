//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_MOUNT_MODEL_H
#define FASTDRL_MOUNT_MODEL_H

#include "../base.h"

class MountModel : public MujocoXMLModel {

public:
    MountModel(string fName, int idn);

    string namingPrefix() override { return "mount" + std::to_string(idn) + "_"; }

    std::array<float, 4> contactGeomRGBA() override;


};


#endif //FASTDRL_MOUNT_MODEL_H
