//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_RETHINK_MOUNT_H
#define FASTDRL_RETHINK_MOUNT_H

#include "mount_model.h"
#include "../../utils/mjcf_utils.h"
class RethinkMount : public MountModel{

public:
    RethinkMount(int idn=0): MountModel(mjcfUtils::xmlPathCompletion("mounts/rethink_mount.xml"), idn) {}

    torch::Tensor topOffset() override { return torch::tensor({0, 0, -0.01}); };

    double horizontalRadius() override { return 0.25; };

};


#endif //FASTDRL_RETHINK_MOUNT_H
