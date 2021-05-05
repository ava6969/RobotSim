//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_SINGLE_ARM_H
#define FASTDRL_SINGLE_ARM_H

#include "../utils/transforms.h"
#include "manipulator.h"
#include "../models/grippers/gripper_model.h"
#include "../models/robots/manipulator_model.h"

class SingleArm : public Manipulator {

protected:
    string controller;
    ControllerConfig controllerConfig;
    string gripperType;
    bool hasGripper;
    std::unique_ptr<GripperModel> gripper;
    std::vector<string> gripperJoints;
    torch::Tensor refGripperJointPosIndexes, refGripperJointVelIndexes, refJointGripperActuatorIndexes, eefRotOffset;
    string eefSiteID, eefCylinderId;
    torch::Tensor torques;

    torch::Tensor recentEEForceTorques, recentEEPose, recentEEVel, recentEEAcc;
    RingBuffer recentEEVelBuffer;



public:
    explicit SingleArm(RobotOptions const& options,
                       ControllerConfig const& controllerConfig,
                       string  gripperType="default");

    void loadController() override;

    template<class MountType, class RobotModelType, class GripperType>
    void loadModel()
    {
        robotModel = make_unique<RobotModelType>(name, idn);
//        if(mountType == "default") return default gripper
//            robotModel->addMount(std::unique_ptr<MujocoXMLModel>(robotModel->defaultMount()));
//        else:
        robotModel->addMount(make_unique<MountType>(idn));

        if(initQPos.sizes().empty())
        {
            initQPos = robotModel->initQPos();
        }

        if(hasGripper)
        {
            if(gripperType == "default")
            {
                //
            }else
            {

            }
            gripper = std::make_unique<GripperType>(gripperType, idn);
        }else
            gripper = std::make_unique<GripperType>("", idn);

        eefRotOffset = transform::quatMultiply(manRobotModel->getHandRotationOffset().at("single"),gripper->getRotationOffset());
        manRobotModel->addGripper(gripper.get());
    }

    void reset(bool deterministic=false) override;

    void setReference();



};


#endif //FASTDRL_SINGLE_ARM_H
