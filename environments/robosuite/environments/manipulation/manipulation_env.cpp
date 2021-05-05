//
// Created by dewe on 4/24/21.
//

#include "manipulation_env.h"

ManipulationEnv::ManipulationEnv(RobotEnvOptions _options): RobotEnv(std::move(_options))
{
    auto robots = this->baseOptions.robots;
    auto numRobots = robots.size();

    auto gripperTypes = input2List(this->baseOptions.gripper_types, numRobots);

    robot_config = { //
             };


}

std::set<string> ManipulationEnv::visualizations() {
    auto visSet = RobotEnv::visualizations();
    visSet.insert("grippers");
    return visSet;
}

bool ManipulationEnv::checkGrasp(MujocoModel *gripper, struct GripperModel *objectGeoms) {
    return false;
}

torch::Tensor
ManipulationEnv::gripperToTarget(MujocoModel *gripper, MujocoModel *target, string targetType, bool returnDistance) {
    return torch::Tensor();
}

void ManipulationEnv::visualizeGripperToTarget(MujocoModel *gripper, MujocoModel *target, const string &targetType) {

}

void ManipulationEnv::visualizeGRipperToTarget(MujocoModel *gripper, const string &target, const string &targetType) {

}

void ManipulationEnv::checkRobotConfiguration(const vector<string> &robots) {

}
