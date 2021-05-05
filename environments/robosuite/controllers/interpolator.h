//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_INTERPOLATOR_H
#define FASTDRL_INTERPOLATOR_H

#include "string"
#include "torch/torch.h"
#include "vector"

using std::vector;
using std::string;

class Interpolator {

public:
    virtual torch::Tensor getInterpolatedGoal() = 0;
    virtual void setGoal(torch::Tensor const& goal)=0;
};

class LinearInterpolator: public Interpolator
{
private:
    int nDim, controllerFreq, policyFreq;
    float rampRatio;
    bool useDeltaGoal;
    std::string oriInterpolate;
    double totalSteps;
    int order;
    int step;
    torch::Tensor goal, start;

public:
    LinearInterpolator(int nDim,
                       int controllerFreq,
                       int policyFreq,
                       float rampRatio=0.2,
                       bool useDeltaGoal= false,
                       std::string oriInterpolate="");

    void setStates(int nDim=-1, const string& oriInterpolate="");
    void setGoal(torch::Tensor const& goal) override;
    torch::Tensor getInterpolatedGoal() override;
};


#endif //FASTDRL_INTERPOLATOR_H
