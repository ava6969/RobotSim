//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_SINGLE_ARM_ENV_H
#define FASTDRL_SINGLE_ARM_ENV_H

#include "manipulation_env.h"
class SingleArmEnv : public ManipulationEnv{

public:

    void loadModel() override;

    void checkRobotConfiguration(const vector<string> &robots) override;

    torch::Tensor eefXPos();

    torch::Tensor eefXMat();

    torch::Tensor eefXQuat();
};


#endif //FASTDRL_SINGLE_ARM_ENV_H
