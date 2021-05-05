//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_JOINT_POS_H
#define FASTDRL_JOINT_POS_H

#include "base_controller.h"
#include "interpolator.h"

static const vector<string> IMPEDANCE_MODES = {"fixed", "variable", "variable_kp"};

class JointPositionController : public BaseController {

   int controlDim;
    Space::Range<torch::Tensor> positionLimits;
    torch::Tensor kp, kd, kpMin, kpMax,
   dampingRatioMin, dampingRatioMax;
    std::unique_ptr<Interpolator> interpolator;
    string impedanceMode;

    int controlFreq;
    torch::Tensor goalQPos;

public:
    JointPositionController( MjSim* sim,
                             CtrlParameters const& params,
                             std::unique_ptr<Interpolator> && interpolator);

    void setGoal(torch::Tensor const& action, torch::Tensor  setqPos);

    torch::Tensor  runController() override;

    void resetGoal() override;

    std::tuple<torch::Tensor, torch::Tensor>  controlLimits();

    inline std::string name() override { return "JOINT_POSITION"; }

};


#endif //FASTDRL_JOINT_POS_H
