//
// Created by dewe on 4/24/21.
//

#include "base_controller.h"
#include "../mj_sim.h"

torch::Tensor BaseController::nums2Array(const vector<int>& nums, int dims)
{
    return torch::from_blob((void *) nums.data(), {static_cast<long>(nums.size())});
}

torch::Tensor BaseController::nums2Array(float nums, int dims)
{
    return torch::ones({dims}) * nums;
}

void BaseController::updateInitialJoints(const torch::Tensor &initialJoints)
{
    this->initialJoint = initialJoints;
    update(true);
    initialEEPos = eePos.value();
    initialEEOriMat = eeOriMat.value();
}

void BaseController::update(bool force)
{
    if(newUpdate || force)
    {
        sim->forward();
        auto eefID = sim->siteName2ID(eefName);
        eePos = torch::from_blob(&sim->data->site_xpos[eefID * m3], {3});
        eeOriMat = torch::from_blob(&sim->data->site_xmat[eefID * m9], {9}).view({3, 3});
        std::tie(eePosVel, eeOriVel) = sim->site_xvel(eefID);

        jointPos = sim->qpos()[jointIndex];
        jointVel = sim->qvel()[qVelIndex];

        std::tie(jPos, jOri) = sim->get_site_jac(eefID);
        jPos = jPos.view({3, -1}).gather(1, qVelIndex);
        jPos = jOri.view({3, -1}).gather(1, qVelIndex);
        jFull = torch::vstack({jPos, jOri});

        massMatrix = sim->massMatrix()[qVelIndex].gather(1, qVelIndex);

        newUpdate = false;

    }

}

torch::Tensor BaseController::scaleAction(const torch::Tensor &action)
{
    if(actionScale.has_value())
    {
        actionScale = ((outputMax - outputMin).abs() / (inputMax - inputMin).abs());
        actionOutputTransform = (outputMax + outputMin) / 2.0;
        actionInputTransform = (inputMax + inputMin) / 2.0;
    }

    torch::Tensor _action = torch::max(torch::min(action, inputMax), inputMin);
    torch::Tensor transformedAction = (_action - actionInputTransform.value()) *
            actionScale.value() +
            actionOutputTransform.value();

    return transformedAction;
}

BaseController::BaseController(MjSim *sim, const string &eefName,
                               const unordered_map<string, torch::Tensor> &jointIndexes,
                               const Space::Range<torch::Tensor>  &actuatorRange):
                               actuatorMin(actuatorRange.low),
                               actuatorMax(actuatorRange.high),
                               sim(sim),
                               jointIndex(jointIndexes.at("joints")),
                               qPosIndex(jointIndexes.at("qpos")),
                               qVelIndex(jointIndexes.at("qvel")),
                               jointDim(jointIndexes.at("joints").sizes().size()){

    sim->forward();

    this->update();
    initialJoint = jointPos.value();
    initialEEPos = eePos.value();
    initialEEOriMat = eeOriMat.value();

}

torch::Tensor BaseController::clipTorques(const torch::Tensor &_torques)
{
    return torch::max(torch::min(_torques, actuatorMax), actuatorMin);
}

torch::Tensor BaseController::torqueCompensation() { return sim->qfrc_bias()[qVelIndex]; }