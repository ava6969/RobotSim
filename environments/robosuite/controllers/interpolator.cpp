//
// Created by dewe on 4/24/21.
//

#include "interpolator.h"
#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include "../utils/transforms.h"

LinearInterpolator::LinearInterpolator(int nDim, int controllerFreq, int policyFreq, float rampRatio, bool useDeltaGoal,
                                       std::string oriInterpolate) :nDim(nDim), controllerFreq(controllerFreq),
                                       policyFreq(policyFreq), rampRatio(rampRatio), useDeltaGoal(useDeltaGoal),
                                                                    oriInterpolate(std::move(oriInterpolate)),order(1), step(0)
                                       {
    totalSteps = std::ceil(rampRatio * (double)controllerFreq / policyFreq);
    setStates(nDim, oriInterpolate);
}

void LinearInterpolator::setGoal(const torch::Tensor & _goal)
{
    if(goal.size(0) != nDim)
    {
        std::cout << goal << std::endl;
        std::stringstream error_msg;
        error_msg << "LinearInterpolator: Input size wrong for goal; got " <<  goal.size(0) << " needs to be " << nDim << std::endl;
        throw std::length_error(error_msg.str());
    }
    this->start = _goal;
    this->goal = _goal;
    this->step = 0;

}

void LinearInterpolator::setStates(int _nDim, const string& _oriInterpolate)
{
    this->nDim = nDim == -1 ? this->nDim :  _nDim;
    this->oriInterpolate = oriInterpolate.empty() ? this->oriInterpolate : _oriInterpolate;

    if(!this->oriInterpolate.empty())
    {
        start = oriInterpolate == "euler" ? torch::zeros({3}) : torch::tensor(vector<float>{0, 0, 0, 1});

    } else
    {
        start = torch::zeros(this->nDim);
    }
    goal = start;
}

torch::Tensor LinearInterpolator::getInterpolatedGoal()
{
    torch::Tensor x = start;
    torch::Tensor xCurrent;
    if (! oriInterpolate.empty()) {
        torch::Tensor _goal = goal;
        if (oriInterpolate == "euler") {

            x = transform::mat2quat(transform::euler2mat(x));
            _goal = transform::mat2quat(transform::euler2mat(goal));
        }

        xCurrent = transform::quatSlerp(x, _goal, (step + 1) / totalSteps);
        if (oriInterpolate == "euler") {
            xCurrent = transform::mat2euler(transform::quat2mat(xCurrent));
        }
    }
    else
    {
        auto dx = (goal - x) / (totalSteps - step);
        xCurrent = x + dx;
    }

    if(step < totalSteps - 1)
    {
        step++;
    }

    return xCurrent;
}
