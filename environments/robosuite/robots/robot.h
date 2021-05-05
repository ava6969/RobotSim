//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_ROBOT_H
#define FASTDRL_ROBOT_H

#include "../utils/buffers.h"
#include "string"
#include "variant"
#include "vector"
#include "../utils/buffers.h"
#include "../utils/globals.h"
#include "torch/torch.h"
#include "../models/robots/robot_model.h"

using std::string;
using std::vector;
using std::map;

struct Noise
{
    enum Type{
        uniform,
        gaussian,
        none,
        default_
    };

    Type type{Type::none};
    float magnitude{0.f};
};
using MultiDict = std::unordered_map<string, std::variant<string, int, float, vector<float>, vector<string>> >;
using ControllerConfig = std::unordered_map<string, std::variant<string, int, float, vector<float>, vector<string>, MultiDict, MjSim*>>;


struct RobotOptions
{
    string robotType;
    int idn=0;
    torch::Tensor  initialQPos={};
    Noise initializationNoise={};
    string mountType="default";
    int controlFreq=20;
};

class Robot {

protected:

    string robotType;
    int idn;
    std::unique_ptr<RobotModel> robotModel;
    int controlFreq;
    string mountType;
    Noise initNoise;
    torch::Tensor initQPos;
    vector<string> robot_joints;
    torch::Tensor basePos, baseOri;
    DeltaBuffer recentQPos, recentActions, recentTorques;
    string name;

public:

    torch::Tensor refJointIdx, refJointPosIdx, refJointActuatorIdx, refJointVelIdx;
    struct MjSim* mjSim;

    explicit Robot(RobotOptions const& options);

    virtual void loadController() = 0;

    RobotModel* model() const { return robotModel.get();}

    void resetSim(struct MjSim* mjSim);

    virtual void reset(bool deterministic=false);

    void setupReferences();

    map<string, class Observable> setupObservables();

    virtual void control(torch::Tensor const& action, bool policyStep=false) = 0;

    bool checkQLimit();

    virtual void visualize(map<string, bool> visSettings);

    virtual globs::RangeTorch actionLimits();

    virtual globs::RangeTorch torqueLimits();

    virtual int actionDim();

    virtual int dof();

    torch::Tensor poseInBaseFromName(string const& name);

    void setRobotJointPosition(torch::Tensor const& jPos);

    torch::Tensor jsEnergy();

    torch::Tensor jointPositions();

    torch::Tensor jointVelocities();

    torch::Tensor jointIndexes();

    torch::Tensor getSensorMeasurement(string const& name);

    friend class Sensor;


};


#endif //FASTDRL_ROBOT_H
