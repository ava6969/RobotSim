//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_MANIPULATOR_MODEL_H
#define FASTDRL_MANIPULATOR_MODEL_H

#include "../grippers/gripper_model.h"
#include "robot_model.h"

class ManipulatorModel : public RobotModel{

protected:
    std::unordered_map<string, GripperModel*> grippers;
    unordered_map<string, torch::Tensor> handRotationOffset;
public:
    ManipulatorModel(string fName, int idn);

    void initialize() override;

    std::unordered_map<string, GripperModel*> getGrippers() const { return grippers; }

    void addGripper(GripperModel* gripper, string armName);

    void addGripper(GripperModel* gripper) { addGripper(gripper, eefName().at("single").front() ); };

    unordered_map<string, vector<string>> eefName() { return correctNaming(_eefName()); }

    inline unordered_map<string, vector<string>> _importantSites() override { return {{"ee", {"ee"}},
                                                                              {"ee_x", {"ee_x"}},
                                                                              {"ee_y", {"ee_y"}},
                                                                              {"ee_z", {"ee_z"}}};
    }

    virtual unordered_map<string, vector<string>> _eefName() { return {{"single", {"right_hand"}}}; }

    virtual MujocoXMLModel* defaultGripper() = 0;

    virtual string armType() = 0;

    unordered_map<string, torch::Tensor> getHandRotationOffset() { return handRotationOffset; };

};


#endif //FASTDRL_MANIPULATOR_MODEL_H
