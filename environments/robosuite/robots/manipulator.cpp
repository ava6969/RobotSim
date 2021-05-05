//
// Created by dewe on 4/24/21.
//

#include "manipulator.h"
#include "../mj_sim.h"
#include "../models/grippers/gripper_model.h"
#include "../models/robots/manipulator_model.h"
#include "../models/robots/robot_model.h"

void Manipulator::gripAction(GripperModel *gripper, float gripperAction) {

    // find way to avoid these loops
    torch::Tensor actuatorIdx = torch::empty({static_cast<long>(gripper->actuators().size())});
    int i = 0;
    for(auto const& actuator: gripper->actuators())
    {
        actuatorIdx[i++] = mjSim->actName2ID(actuator);
    }
    auto gripperActionActual = gripper->formatAction(torch::tensor(gripperAction));
    auto ctrlRange = mjSim->rawActCtrlRange()[actuatorIdx];
    auto bias = 0.5 * (ctrlRange.index({":", 1}) + ctrlRange.index({":", 0}) );
    auto weight = 0.5 * (ctrlRange.index({":", 1}) + ctrlRange.index({":", 0}) );
    auto appliedGripperAction =  bias + weight * gripperActionActual;
    mjSim->actCtrl(actuatorIdx, appliedGripperAction);

    manRobotModel = dynamic_cast<ManipulatorModel*>(robotModel.get());
    assert(manRobotModel);
}

void Manipulator::visualize(map<string, bool> _visSettings)
{
    visualizeGrippers(_visSettings["grippers"]);
}


int Manipulator::dof()
{
    auto _dof = 0;

    for(auto& gripper : manRobotModel->getGrippers())
    {
        _dof += gripper.second->dof();
    }
    return _dof;
}
