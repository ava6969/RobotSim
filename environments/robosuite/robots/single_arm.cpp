//
// Created by dewe on 4/24/21.
//

#include "single_arm.h"
#include "../mj_sim.h"



//SingleArm::SingleArm(const RobotOptions &options,
//                     const ControllerConfig &controllerConfig,
//                     string _gripperType) :
//                     Manipulator(options),gripperType(std::move(_gripperType)),
//                     hasGripper(!gripperType.empty()),controllerConfig(controllerConfig)
//                     {
//}

//void SingleArm::loadController() {
//
//    if(controllerConfig.empty())
//    {
//
//    }
//
//    controllerConfig["robot_name"] = name;
//    controllerConfig["sim"] = mjSim;
//    controllerConfig["eef_name"] = gripper->importantSites().at("grip_site");
//    controllerConfig["eef_rot_offset"] = eefRotOffset;
//    controllerConfig["joint_indexes"] = {
//
//            {"joints", jointIndexes()},
//            {"qpos", refJointPosIdx()},
//            {"qvel", refJointVelIdx()}
//    };
//    controllerConfig["actuator_range"] = torqueLimits();
//    controllerConfig["policy_freq"] = controlFreq;
//    controllerConfig["ndim"] = robot_joints.size();
//
//    controller = controllerFactory(controllerConfig["type"], controllerConfig);
//
//}
//

void SingleArm::reset(bool deterministic) {
    Robot::reset(deterministic);
    Manipulator::reset(deterministic);

    if(!deterministic)
    {
        if(hasGripper)
        {
            mjSim->setQPos(refGripperJointPosIndexes, gripper->initQPos);
        }
    }
    controller->updateBasePose(basePos, baseOri);

    recentEEForceTorques = DeltaBuffer(6);
    recentEEPose = DeltaBuffer(7);
    recentEEVel = DeltaBuffer(6);
    recentEEVelBuffer = RingBuffer(6, 10);
    recentEEAcc = DeltaBuffer(6);
}

void SingleArm::setReference() {



}
