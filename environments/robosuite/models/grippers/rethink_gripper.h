//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_RETHINK_GRIPPER_H
#define FASTDRL_RETHINK_GRIPPER_H

#include "../../utils/mjcf_utils.h"
#include "gripper_model.h"

class RethinkGripperBase : public GripperModel{


public:
    explicit RethinkGripperBase(int idn=0): GripperModel(mjcfUtils::xmlPathCompletion("grippers/rethink_gripper.xml"), idn){}

    torch::Tensor formatAction(torch::Tensor const& action ) override { return action; }

    torch::Tensor initQPos() override { return torch::tensor({0.020833, -0.020833}); }

    unordered_map<string, vector<string>> _importantGeoms() override
    {
        return {
                {"left_finger", {"l_finger_g0", "l_finger_g1", "l_fingertip_g0", "l_fingerpad_g0"}},
                {"right_finger", {"r_finger_g0", "r_finger_g1", "r_fingertip_g0", "r_fingerpad_g0"}},
                {"left_fingerpad", {"l_fingerpad_g0"}},
                {"right_fingerpad", {"r_fingerpad_g0"}}
        };
    }


};

class RethinkGripper : public RethinkGripperBase{

public:

    explicit RethinkGripper(int idn=0): RethinkGripperBase(idn){}

    torch::Tensor formatAction(torch::Tensor const& action ) override { return action; }

    float speed() override { return 0.01;}

    int dof() override {return 1; }

};



#endif //FASTDRL_RETHINK_GRIPPER_H
