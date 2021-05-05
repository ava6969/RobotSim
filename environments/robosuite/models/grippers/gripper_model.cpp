//
// Created by dewe on 4/27/21.
//

#include "../../utils/mjcf_utils.h"
#include "gripper_model.h"

unordered_map<string, vector<string>>  GripperModel::_importantSensors() {
    return {{"force_ee", {"force_ee"}},
            {"torque_ee", {"torque_ee"}}};
}

unordered_map<string, vector<string>> GripperModel::_importantGeoms() {
    return {{"left_fingers", {}},
            {"right_finger", {}},
            {"left_fingerpad", {}},
            {"right_fingerpad", {}}};
}

unordered_map<string, vector<string>>  GripperModel::_importantSites() {
    return {{"grip_site", {"grip_site"}},
            {"grip_cylinder", {"grip_site_cylinder"}}};
}

array<float, 4> GripperModel::contactGeomRGBA() {
    return globs::GRIPPER_COLLISION_COLOR;
}

double GripperModel::horizontalRadius() {
    return 0;
}

torch::Tensor GripperModel::topOffset() {
    return torch::zeros(3);
}

torch::Tensor GripperModel::bottomOffset() {
    return torch::zeros(3);
}

int GripperModel::dof() {
    return (int)_actuators.size();
}

GripperModel::GripperModel(string fName, int idn) : MujocoXMLModel(fName, idn)
{
    currentAction = torch::zeros({dof()});

    auto attr = worldBody.attribute("quat");
    auto vec_str = attr ? attr.value() : "1 0 0 0";

    auto vec = mjcfUtils::stringToArray<double>(vec_str);
    rotationOffset = torch::tensor(vec).permute({1, 2, 3, 0});
}


string GripperModel::namingPrefix()
{
    return "gripper" + std::to_string(this->idn) + "_";
}

float GripperModel::speed()
{
    return 0.f;
}
