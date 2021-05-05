//
// Created by dewe on 4/24/21.
//

#include "robot_model.h"
#include "../../utils/mjcf_utils.h"
#include "../../utils/transforms.h"

RobotModel::RobotModel(string fName, int idn): MujocoXMLModel(fName, idn)
{

    MujocoXMLModel::initialize();
    cameras = getElementNames(worldBody, "camera");

    auto t = 0.1 * torch::ones(dof());
    vector<float> one(dof(), 0.1);
    vector<float> arm(dof());

    for(int i = 0; i < dof();i++)
    {
        arm[i] = 5.f / float(i + 1);
    }

    setJointAttribute("frictionless", one, false);
    setJointAttribute("damping", one, false);
    setJointAttribute("armature", arm, false);
}

void RobotModel::setBaseOri(const vector<float> &rot) {

    // use mujoc function for conversion
    auto _rot = transform::mat2quat(transform::euler2mat(torch::tensor(rot))).permute({3, 0, 1, 2});
    elements["root_body"].front().attribute("root_body").set_value(mjcfUtils::arrayToString_(_rot).c_str());

}

void RobotModel::setJointAttribute(const string &attrib, const vector<float> &rot, bool force) {

    std::stringstream error;
    error << "Error setting joint attributes: Values must be same size as joint dimension. Got " << rot.size() << " expected " << dof() << std::endl;
    auto rot_ = torch::tensor(rot);
    assert(rot.size() == elements["joints"].size() && error.str());
    int i = 0;
    while(i < elements["joints"].size())
    {
        auto& joint = elements["joints"][i];
        if(force && joint.attribute(attrib.c_str()))
        {
            joint.attribute(attrib.c_str()).set_value(mjcfUtils::arrayToString_(rot_[i]).c_str());
        }
        i++;
    }


}

void RobotModel::addMount(std::unique_ptr<MujocoXMLModel>& _mount)
{
    if(mount)
        throw std::runtime_error("Mount already added for this robot!");

    auto offset = baseOffset() - _mount->topOffset();
    _mount->getElement()["root_body"].front().attribute( "pos").set_value(mjcfUtils::arrayToString_(offset).c_str());

    merge(_mount, rootBody());

    this->mount = std::move(_mount);

    cameras = getElementNames(worldBody, "camera");

}

torch::Tensor RobotModel::bottomOffset()  {
    return mount ? (mount->bottomOffset() - mount->topOffset()) + _base_offset : _base_offset;
}

void RobotModel::setBasePos(const torch::Tensor &pos)
{
    elements["robot_body"].front().attribute("pos").set_value(mjcfUtils::arrayToString_(pos - bottomOffset()).c_str());
}

std::array<float, 4> RobotModel::contactGeomRGBA() {
    return globs::ROBOT_COLLISION_COLOR;
}
