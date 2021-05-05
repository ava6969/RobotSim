//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_MANIPULATOR_H
#define FASTDRL_MANIPULATOR_H

#include "robot.h"
#include "variant"

class Manipulator : public Robot {

protected:
    class ManipulatorModel* manRobotModel;

public:
    explicit Manipulator(RobotOptions const& options): Robot(options){}

    virtual void gripAction(class GripperModel* gripper, float gripperAction);

    void visualize(map<string, bool> visSettings) override;

    virtual void visualizeGrippers(bool visible) = 0 ;

    int dof() override;

    virtual std::variant<float, unordered_map<string, float>> eeFtIntegral() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> eeForce() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> eeTorque() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handPose() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handQuat() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handTotalVelocity() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handPos() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handOrn() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handVel() = 0;
    virtual std::variant<torch::Tensor, unordered_map<string, torch::Tensor>> handAngVel() = 0;


};


#endif //FASTDRL_MANIPULATOR_H
