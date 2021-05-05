//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_SAWYER_ROBOT_H
#define FASTDRL_SAWYER_ROBOT_H

#include "manipulator_model.h"
#include "../../utils/mjcf_utils.h"
#include "../mounts/rethink_mount.h"
#include "../grippers/rethink_gripper.h"
class Sawyer : public ManipulatorModel
{

public:
    explicit Sawyer(int idn): ManipulatorModel(mjcfUtils::xmlPathCompletion("robots/sawyer/robot.xml"), idn=idn)
    {}

    inline MujocoXMLModel* defaultMount(string name, int id) { new MountModel(std::move(name), id);}
    inline MujocoXMLModel* defaultGripper(int id) { new class RethinkGripper(id); }
    inline string defaultControllerConfig() override { return "default_sawyer"; }
    inline torch::Tensor initQPos() override { return torch::tensor({0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161});}

    inline map<string, std::variant<std::function<vector<float>(float)>, vector<float>> baseXPosOffset(){
        return {{"bins", {-0.5, -0.1, 0}}, {"empty", {-0.6, 0, 0}},
                {"table", [](float lenght) { return {static_cast<float>(-0.16 - lenght/2) , 0, 0}; }};
    }

    inline torch::Tensor topOffset() override { return {0, 0, 1.0}; }
    inline float horizontalRadius() override { return 0.5; }
    inline string armType() override { return "single"; }

}


#endif //FASTDRL_SAWYER_ROBOT_H
