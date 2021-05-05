//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_GRIPPER_MODEL_H
#define FASTDRL_GRIPPER_MODEL_H

#include "../base.h"
#include "array"

using std::array;

class GripperModel : public MujocoXMLModel {

protected:
    torch::Tensor currentAction;

public:

    GripperModel(string fName, int idn);

    virtual torch::Tensor formatAction(torch::Tensor const& action ) = 0;

    string namingPrefix() override;

    virtual float speed();

    virtual int dof();

    torch::Tensor bottomOffset() override;

    torch::Tensor topOffset() override;

    double horizontalRadius() override;

    array<float, 4> contactGeomRGBA() override;

    unordered_map<string, vector<string>> _importantSites() override;

    unordered_map<string, vector<string>>  _importantGeoms() override;

    unordered_map<string, vector<string>>  _importantSensors() override;

    virtual torch::Tensor initQPos() = 0;

};


#endif //FASTDRL_GRIPPER_MODEL_H
