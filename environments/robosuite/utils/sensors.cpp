//
// Created by dewe on 4/30/21.
//
#include "sensors.h"
#include "../robots/robot.h"
#include "../mj_sim.h"
#include "mjcf_utils.h"

torch::Tensor Sensor::get(unordered_map<string, torch::Tensor> const& obsCache)=0;

Sensor::Sensor(string const& modality, string const& preCompute_, Robot* owner_) :
    modality(modality),
    preCompute(preCompute_),
    owner(owner_)
{

}


torch::Tensor JointPos::get(unordered_map<string, torch::Tensor> const& obsCache)
{
    return  owner->mjSim->qpos()[owner->refJointPosIdx];
}

JointPos::JointPos(const string &modality, const string &preCompute, struct Robot *owner) : Sensor(modality, preCompute,
                                                                                                   owner) {

}

torch::Tensor JointPosCos::get(unordered_map<string, torch::Tensor> const& obsCache)
{
    return mjcfUtils::found(obsCache, preCompute) ? obsCache.at(preCompute).cos() : torch::zeros(owner->model()->dof());
}

JointPosCos::JointPosCos(const string &modality, const string &preCompute, struct Robot *owner) : Sensor(modality,
                                                                                                         preCompute,
                                                                                                         owner) {

}


torch::Tensor JointPosSin::get(unordered_map<string, torch::Tensor> const& obsCache)
{
    return mjcfUtils::found(obsCache, preCompute) ? obsCache.at(preCompute).sin() : torch::zeros(owner->model()->dof());
}

JointPosSin::JointPosSin(const string &modality, const string &preCompute, struct Robot *owner) : Sensor(modality,
                                                                                                         preCompute,
                                                                                                         owner) {

}


torch::Tensor JointVel::get(unordered_map<string, torch::Tensor> const& obsCache)
{
    return owner->mjSim->qvel()[owner->refJointPosIdx];
}

JointVel::JointVel(const string &modality, const string &preCompute, struct Robot *owner) : Sensor(modality, preCompute,
                                                                                                   owner) {

}
