//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_MANIPULATION_ENV_H
#define FASTDRL_MANIPULATION_ENV_H

#include "../robot_env.h"

class ManipulationEnv : public RobotEnv{

public:
    ManipulationEnv(RobotEnvOptions options);

    std::set<string> visualizations() override;

    bool checkGrasp(MujocoModel* gripper, class GripperModel* objectGeoms);

    torch::Tensor gripperToTarget(MujocoModel* gripper, MujocoModel* target, string targetType="body", bool returnDistance=false);

    void visualizeGripperToTarget(MujocoModel* gripper, MujocoModel* target, string const& targetType= "body");

    void visualizeGRipperToTarget(MujocoModel* gripper, string const& target, string const& targetType="body");

    void checkRobotConfiguration(const vector<string> &robots) override;



};


#endif //FASTDRL_MANIPULATION_ENV_H
