//
// Created by dewe on 4/30/21.
//

#ifndef FASTDRL_SENSORS_H
#define FASTDRL_SENSORS_H

#include "functional"
#include "torch/torch.h"
#include "unordered_map"
#include "memory"
using std::string;
using std::unordered_map;

struct Sensor
{
    string modality, preCompute;
    class Robot* owner;
    Sensor(string const& modality, string const& preCompute, class Robot* owner);
    virtual torch::Tensor get(unordered_map<string, torch::Tensor> const& obsCache) = 0;

};

struct JointPos : Sensor {
    JointPos(string const& modality, string const& preCompute, class Robot* owner);
    torch::Tensor get(unordered_map<string, torch::Tensor> const& obsCache) override;
};

struct JointPosCos : Sensor {
    JointPosCos(string const& modality, string const& preCompute, class Robot* owner);
    torch::Tensor get(unordered_map<string, torch::Tensor> const& obsCache) override;
};

struct JointPosSin : Sensor {
    JointPosSin(string const& modality, string const& preCompute, class Robot* owner);
    torch::Tensor get(unordered_map<string, torch::Tensor> const& obsCache) override;
};
struct JointVel : Sensor {
    JointVel(string const& modality, string const& preCompute, class Robot* owner);
    torch::Tensor get(unordered_map<string, torch::Tensor> const& obsCache) override;
};


#endif //FASTDRL_SENSORS_H
