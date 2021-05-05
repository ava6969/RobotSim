//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_NULL_MOUNT_H
#define FASTDRL_NULL_MOUNT_H

#include "mount_model.h"

class NullMount : public MountModel {

public:
    NullMount(int idn);

    torch::Tensor topOffset() override { return torch::zeros({3}); }

    double horizontalRadius() override { return 0.f; }
};


#endif //FASTDRL_NULL_MOUNT_H
