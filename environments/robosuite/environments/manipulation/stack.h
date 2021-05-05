//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_STACK_H
#define FASTDRL_STACK_H

#include "single_arm_env.h"

struct StackOptions
{
    RobotEnvOptions base_opt;
    std::array<float, 3> tableFullSize = {0.8, 0.8, 0.05};
    std::array<float, 3> tableFriction = {0.8, 0.8, 0.05};
};

class Stack : public SingleArmEnv{

public:
    explicit Stack(StackOptions options);

    float reward(torch::Tensor const& action) override;

    std::tuple<float, float, float> stagedRewards();

    void loadModel() override;

    void setupReferences() override;

    void resetInternal() override;

    void setupObservables() override;

    bool checkSuccess() override;

    void visualize(map<string, bool> const& visSettings) override;



};


#endif //FASTDRL_STACK_H
