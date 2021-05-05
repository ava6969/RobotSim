//
// Created by dewe on 4/24/21.
//

#pragma once

#include "../base.h"
#include "map"


using std::map;
class RobotModel : public MujocoXMLModel{

protected:
    virtual float _horizontalRadius() = 0;
    MujocoXMLModel* mount{};
    vector<string> cameras;

public:
    RobotModel(string fName, int idn);

    void setBasePos(torch::Tensor const& pos);

    void setBaseOri(vector<float>  const& rot);

    void setJointAttribute(string const& attrib, vector<float>  const& rot, bool force);

    void addMount(std::unique_ptr<MujocoXMLModel>& mount);

    string namingPrefix() override { return "robot" + std::to_string(idn) + "_"; }

    int dof() { return _joints.size(); }

    torch::Tensor bottomOffset() override;
    double horizontalRadius() override { return std::max<double>(_horizontalRadius(), mount->horizontalRadius()); }
    std::array<float, 4> contactGeomRGBA() override;

    virtual MujocoXMLModel* defaultMount() = 0;
    virtual string defaultControllerConfig() = 0;
    virtual torch::Tensor initQPos() = 0;
    virtual map<string, vector<float>> baseXPosOffset() = 0;


};
