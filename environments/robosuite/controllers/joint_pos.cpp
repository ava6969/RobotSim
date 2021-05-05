//
// Created by dewe on 4/24/21.
//

#include "../utils/control_utils.h"
#include "joint_pos.h"

void JointPositionController::setGoal(const torch::Tensor &action,
                                      torch::Tensor setqPos)
{
    update();

    auto jntDim = qPosIndex.size(0);
    torch::Tensor delta;
    if(impedanceMode == "variable")
    {
        auto dampingRatio = action.slice(0 , 0, jntDim);
        auto _kp = action.slice(0, jntDim, 2*jntDim);
        auto _delta = action.slice(0, 2*jntDim);
        kp = torch::max(torch::min(kp, kpMax), kpMin);
        kd = 2 * kp.sqrt() *  torch::max(torch::min(dampingRatio, dampingRatioMax), dampingRatioMin);
    }else if (impedanceMode == "variable_kp")
    {
        auto _kp = action.slice(0, 0, jntDim);
        auto _delta = action.slice(0, jntDim );
        kp = torch::max(torch::min(kp, kpMax), kpMin);
        kd = 2 * kp.sqrt();
    }
    else
        delta = action;

    assert(delta.size(0) == jntDim && "Delta qpos must be equal to the robot's joint dimension space!");

    auto scaledDelta = scaleAction(delta);

    goalQPos = ctrlUtils::setGoalPosition(scaledDelta, jointPos.value(),
                                          &positionLimits,
                                          &setqPos);
    if(interpolator)
    {
        interpolator->setGoal(goalQPos);
    }
}

torch::Tensor JointPositionController::runController()
{

    if(goalQPos.sizes().empty())
        setGoal(torch::zeros({controlDim}), {});

    update();

    torch::Tensor desiredQPos;
    if(interpolator)
    {
        // only order 1 currently supported
        desiredQPos = interpolator->getInterpolatedGoal();

    }else
    {
        desiredQPos = goalQPos;
    }

    auto positionError = desiredQPos - jointPos.value();
    auto velPosError = -jointVel.value();
    auto desiredTorque = positionError * kp + velPosError * kd;

    torques = massMatrix->dot(desiredTorque) + torqueCompensation();

    BaseController::runController();

    return torques.value();

}

void JointPositionController::resetGoal() {

    goalQPos = jointPos.value();
    if(interpolator)
        interpolator->setGoal(goalQPos);
}

std::tuple<torch::Tensor, torch::Tensor> JointPositionController::controlLimits()
{
    torch::Tensor low, high;
    if(impedanceMode == "variable")
    {
        low = torch::cat({dampingRatioMin, kpMin, inputMin});
        high = torch::cat({dampingRatioMax, kpMax, inputMax});
    }    if(impedanceMode == "variable")
    {
        low = torch::cat({ kpMin, inputMin});
        high = torch::cat({ kpMax, inputMax});
    }else {
        low = torch::cat({inputMin});
        high = torch::cat({inputMax});
    }

    return {low, high};
}

JointPositionController::JointPositionController(MjSim *sim, const CtrlParameters &params,
                                                 std::unique_ptr<Interpolator> && interpolator):
                                                 interpolator(move(interpolator)),
BaseController(sim, params.eefName, params.jointIndexes, params.actuatorRange)
{

    controlDim = jointIndex.size(0);
    inputMax = nums2Array(params.input_max, controlDim);
    inputMin = nums2Array(params.input_min, controlDim);
    outputMax = nums2Array(params.output_max, controlDim);
    outputMin = nums2Array(params.output_min, controlDim);

    positionLimits = params.qpos_limits.value();

    kp = nums2Array(params.kp, controlDim);
    kd = 2 * kp.sqrt() * params.damping_ratio;


    kpMin = nums2Array(params.kp_limits[0], controlDim);
    kpMax = nums2Array(params.kp_limits[1], controlDim);
    dampingRatioMin = nums2Array(params.damping_ratio_limits[0], controlDim);
    dampingRatioMax = nums2Array(params.damping_ratio_limits[1], controlDim);

    assert(mjcfUtils::found(IMPEDANCE_MODES, params.impedance_mode));
    impedanceMode = params.impedance_mode;

    if(impedanceMode == "variable")
    {
        controlDim *= 3;
    }else if (impedanceMode == "variable_kp")
        controlDim *= 2;

    controlFreq = params.policyFreq;


}

