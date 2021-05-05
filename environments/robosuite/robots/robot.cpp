//
// Created by dewe on 4/24/21.
//

#include "../utils/transforms.h"
#include "robot.h"
#include "../utils/sensors.h"
#include "../mj_sim.h"
#include "../utils/observables.h"


int Robot::dof() {
    auto _dof = robotModel->dof();
    return _dof;
}

Robot::Robot(RobotOptions const& options):name(options.robotType),idn(options.idn), controlFreq(options.controlFreq),
             mountType(options.mountType),
             initNoise(options.initializationNoise),
             initQPos(options.initialQPos){

    if(initNoise.type == Noise::Type::none)
    {
        initNoise = Noise{Noise::Type::gaussian, 0.0};
    }else if (initNoise.type == Noise::Type::default_)
    {
        initNoise = Noise{Noise::Type::gaussian, 0.02};
    }

}

void Robot::resetSim(MjSim * _mjSim) {

    this->mjSim = _mjSim;
}

void Robot::reset(bool deterministic) {

    torch::Tensor noise;
    if(!deterministic)
    {
        switch(initNoise.type)
        {
            case Noise::Type::gaussian:
                noise = torch::randn(initQPos.size(0)) * initNoise.magnitude;
                break;
            case Noise::Type::uniform:
                noise = torch::rand(initQPos.size(0)).uniform_(-1, 1) * initNoise.magnitude;
                break;
            default:
                throw std::runtime_error("Error: Invalid noise type specified. Options are 'gaussian' or 'uniform'.");
        }
        initQPos += noise;

        mjSim->setQPos(refJointPosIdx, initQPos);

        loadController();

        recentQPos = DeltaBuffer(jointIndexes().size(0));
        recentActions = DeltaBuffer(actionDim());
        recentTorques = DeltaBuffer(jointIndexes().size(0));

    }
}

void Robot::setupReferences()
{
    robot_joints = robotModel->joints();
    int i = 0;
    for(auto& joint : robot_joints)
    {
        refJointPosIdx[i] = mjSim->getJointQposAddr(joint);
        refJointVelIdx[i] = mjSim->getJointQvelAddr(joint);
        refJointIdx[i] = mjSim->jointName2ID(joint);
    }
    i = 0;
    for(auto& act : robotModel->actuators())
    {
        refJointActuatorIdx[i] = mjSim->actName2ID(act);
    }
}

map<string, Observable> Robot::setupObservables(){

    auto pf = robotModel->namingPrefix();
    auto preCompute = pf + "joint_pos";
    auto modality = pf + "proprio";

    std::vector<std::unique_ptr<Sensor>> sensors = {std::make_unique<JointPos>(modality, preCompute, this),
    std::make_unique<JointPosCos>(modality, preCompute, this),
    std::make_unique<JointPosSin>(modality, preCompute, this),
    std::make_unique<JointVel>(modality, preCompute, this)};

    std::vector<string> names = {"joint_pos", "joint_pos_cos", "joint_pos_cos", "joint_vel"};
    std::vector<bool> actives = {false, true, true, true};
    map<string, Observable> observables;
    for(int i = 0; i < actives.size(); i++)
    {
        observables.insert({pf + names[i],
                            Observable(ObservableOptions{pf + names[i],
                                                         std::move(sensors[i]), {},{},{},
                                                         float(controlFreq), actives[i]}) });
    }

    return observables;
}


bool Robot::checkQLimit()
{
    float tolerance = 0.1;
    torch::Tensor q_, range;
    q_ = mjSim->qpos()[refJointPosIdx];
    range = mjSim->jntRange()[refJointPosIdx];
    for(int i = 0; i < refJointPosIdx.size(0); i++)
    {
        float low = range[i][0].item<float>();
        float high = range[i][1].item<float>();
        float q = q_[i].item<float>();

        if(low != high && !( ( (low + tolerance) < q ) && (q < high - tolerance)))
        {
            printf("Joint limit reached in joint %i", i);
            return true;
        }
    }

    return false;
}

void Robot::visualize(map<string, bool> visSettings) {

    robotModel->setSitesVisibility(mjSim, visSettings.at("robots"));

}



globs::RangeTorch Robot::torqueLimits()
{
    auto[low, high] = mjSim->actCtrlRange();
    return {low, high};
}

int Robot::actionDim()
{
    return actionLimits().low.size(0);
}

torch::Tensor Robot::poseInBaseFromName(const string & name)
{
    auto posInWorld = mjSim->getBodyXPos(name);
    auto rotInWorld = mjSim->getBodyXMat(name).view({3, 3});
    auto poseInWorld = transform::makePose(posInWorld, rotInWorld);

    auto basePosInWorld = mjSim->getBodyXPos(robotModel->rootBody());
    auto baseRotInWorld = mjSim->getBodyXMat(robotModel->rootBody()).view({3, 3});
    auto basePoseInWorld = transform::makePose(basePosInWorld, baseRotInWorld);

    auto worldPoseInBase = transform::poseInv(basePoseInWorld);

    auto poseInBase = transform::pose_in_A_to_pose_in_B(poseInWorld, basePoseInWorld);

    return poseInBase;
}

void Robot::setRobotJointPosition(const torch::Tensor &jPos) {

    mjSim->setQPos(refJointPosIdx, jPos);
    mjSim->forward();

}

torch::Tensor Robot::jsEnergy()
{
    return ((1/ controlFreq) * recentTorques.average()).abs();
}

torch::Tensor Robot::jointPositions() {
    return mjSim->qpos()[refJointPosIdx];
}

torch::Tensor Robot::jointVelocities() {
    return mjSim->qvel()[refJointVelIdx];
}

torch::Tensor Robot::jointIndexes() {
    return refJointPosIdx;
}

torch::Tensor Robot::getSensorMeasurement(const string & _name)
{
    int sensorIdx = mjSim->sensorDim().slice(0,0, mjSim->sensorName2ID(_name)).sum().item<int>();
    int sensorDim = mjSim->sensorDim()[mjSim->sensorName2ID(_name)].item<int>();
    return mjSim->sensorData().slice(0, sensorIdx, sensorIdx + sensorDim);
}
