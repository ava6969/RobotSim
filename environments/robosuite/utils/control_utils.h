//
// Created by dewe on 4/26/21.
//

#ifndef FASTDRL_CONTROL_UTILS_H
#define FASTDRL_CONTROL_UTILS_H

#include "robosuite/utils/space.h"
#include "torch/torch.h"

namespace ctrlUtils
{
    torch::Tensor setGoalPosition(torch::Tensor const& delta,
                                  torch::Tensor const& currentPosition,
                                  Space::Range<torch::Tensor>* positionLimit,
                                  torch::Tensor* setPos);

    torch::Tensor setGoalOrientation( torch::Tensor const& delta,
                                      torch::Tensor const& currentOrientation,
                                      Space::Range<torch::Tensor>* orientationLimit,
                                      torch::Tensor* setOri);
};


#endif //FASTDRL_CONTROL_UTILS_H
