//
// Created by dewe on 4/24/21.
//

#include "single_arm_env.h"
#include "../../utils/transforms.h"

torch::Tensor SingleArmEnv::eefXPos() {
    // get return sim->data->site_xpos()
}

torch::Tensor SingleArmEnv::eefXMat() {
   // siteXMat
}

torch::Tensor SingleArmEnv::eefXQuat() {
    return transform::mat2quat(eefXMat());
}

void SingleArmEnv::checkRobotConfiguration(const vector<string> &robots) {
    ManipulationEnv::checkRobotConfiguration(robots);


}

void SingleArmEnv::loadModel() {
    RobotEnv::loadModel();
    // assert type
}
