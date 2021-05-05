//
// Created by dewe on 4/26/21.
//

#include "control_utils.h"

namespace ctrlUtils
{
    torch::Tensor setGoalPosition(torch::Tensor const& delta,
                                  torch::Tensor const& currentPosition,
                                  Space::Range<torch::Tensor>* positionLimit,
                                  torch::Tensor* setPos)
    {
        int n = currentPosition.size(0);
        torch::Tensor goalPosition;
        if(setPos)
        {
            goalPosition = *setPos;
        }else
        {
            goalPosition = currentPosition + delta;
        }

        if(positionLimit)
        {
            if(positionLimit->low.size(0) != n &&
            positionLimit->high.size(0) != n )
            {
                char buffer[1024];
                std::sprintf(buffer, "Position limit should be shaped (2, %i) "
                                     "but is instead: %li", n, positionLimit->low.size(0));
                throw std::runtime_error(buffer);
            }
            goalPosition = torch::max(positionLimit->low, torch::min(goalPosition, positionLimit->high));
        }

        return goalPosition;
    }

    torch::Tensor setGoalOrientation( torch::Tensor const& delta,
                                      torch::Tensor const& currentOrientation,
                                      Space::Range<torch::Tensor>* orientationLimit,
                                      torch::Tensor* setOri)
    {
        return {};
    }

};