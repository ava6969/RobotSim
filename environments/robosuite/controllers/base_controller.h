//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_BASE_CONTROLLER_H
#define FASTDRL_BASE_CONTROLLER_H


#include "unordered_map"
#include "vector"
#include "../utils/macros.h"
#include "../utils/space.h"
#include "string"

using std::string;
using std::unordered_map;
using std::vector;
using Vect3f = std::array<float, 3>;
using Vect4f = std::array<float, 4>;
using VectXf = vector<float>;
using VectfMap =  unordered_map<string, vector<int>>;


const static unordered_map<string , string> CONTROLLER_INFO = {
        {"JOINT_VELOCITY",  "Joint Velocity"},
        {"JOINT_TORQUE",    "Joint Torque"},
        {"JOINT_POSITION",  "Joint Position"},
        {"OSC_POSITION", "Operational Space Control (Position Only)"},
        {"OSC_POSE",     "Operational Space Control (Position + Orientation)"},
        {"IK_POSE",      "Inverse Kinematics Control (Position + Orientation) (Note: must have PyBullet installed)"}
};


struct CtrlParameters
{
    int nDim;
    string eefName;
    unordered_map<string, torch::Tensor> jointIndexes;
    Space::Range< torch::Tensor> const& actuatorRange;
    float input_max=1;
    float input_min=-1;
    float output_max=0.05;
    float output_min=-0.05;
    float kp=50;
    float damping_ratio={1};
    string impedance_mode="fixed";
    vector<float>kp_limits={0, 300};
    vector<float>damping_ratio_limits={0, 100};
    int policyFreq=20;
    float rampRatio{};
    std::optional<Space::Range< torch::Tensor>> qpos_limits=std::nullopt;
    string interpolator=nullptr;
};

const static vector<std::string> ALL_CONTROLLERS = {"JOINT_VELOCITY", "JOINT_TORQUE", "JOINT_POSITION"};

class BaseController {

protected:
    bool newUpdate{true};
    torch::Tensor actuatorMin, actuatorMax;
    struct MjSim* sim;
    std::optional<torch::Tensor> actionScale, actionInputTransform, actionOutputTransform;
    torch::Tensor  inputMin, inputMax, outputMin, outputMax, controlDim;

    float modelTimeStep = SIMULATION_TIMESTEP;
    string eefName;

    // robot states
    std::optional<torch::Tensor> eePos, eeOriMat, eePosVel, eeOriVel, jointPos, jointVel, massMatrix, torques;
    torch::Tensor initialJoint, initialEEPos, initialEEOriMat,jointIndex, qPosIndex, qVelIndex, jPos, jOri, jFull;
    int jointDim;


public:
    BaseController(struct MjSim* sim,
                   std::string const& eefName,
                   unordered_map<string, torch::Tensor> const& jointIndexes,
                   Space::Range<torch::Tensor> const& actuatorRange);

    virtual torch::Tensor  runController() { newUpdate = true; return {}; }
    torch::Tensor  scaleAction(torch::Tensor const& action);

    void update(bool force=false);
    virtual void updateBasePose(Vect3f const& basePos, Vect4f baseOrientation) {}
    virtual void updateInitialJoints(torch::Tensor const& initialJoints);
    torch::Tensor clipTorques(torch::Tensor const& torques);

    virtual void resetGoal() = 0;

    static torch::Tensor nums2Array(float nums, int dims);
    torch::Tensor nums2Array(const vector<int>& nums, int dims);

    torch::Tensor torqueCompensation();
    std::tuple< torch::Tensor,  torch::Tensor> actuator_limits() { return {actuatorMin, actuatorMax}; }
    std::tuple< torch::Tensor,  torch::Tensor> control_limits() { return {inputMin, inputMax}; }
    virtual std::string name() = 0;


};


#endif //FASTDRL_BASE_CONTROLLER_H
