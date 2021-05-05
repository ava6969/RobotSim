//
// Created by dewe on 4/24/21.
//

#include "manipulator_model.h"
#include "../../utils/mjcf_utils.h"

void ManipulatorModel::addGripper(GripperModel *gripper, string armName) {

    if (armName.empty())
        armName = eefName()["single"].front();
    if (mjcfUtils::found(grippers, armName))
        throw std::runtime_error("Attempts to add multiple grippers to one body");

    merge(gripper, armName);

    grippers[armName] = gripper;

    cameras = getElementNames(worldBody, "camera");

}

ManipulatorModel::ManipulatorModel(string fName, int idn) : RobotModel(std::move(fName), idn) {



}

void ManipulatorModel::initialize()
{
    MujocoXMLModel::initialize();

    if(armType() == "single")
    {
        auto handElement = mjcfUtils::findElements(root, "body", {{"name", eefName()["single"][0]}}, true);
        auto v = handElement.attribute("quat").value();
        v = v ? v : "1 0 0 0";
        handRotationOffset["single"] = torch::tensor(mjcfUtils::stringToArray<float>(v)).permute({1, 2, 3, 0});
    }else
    {
        for(auto& arm : {"right", "left"})
        {
            auto handElement = mjcfUtils::findElements(root, "body", {{"name", eefName()[arm][0]}}, true);
            auto v = handElement.attribute("quat").value();
            v = v ? v : "1 0 0 0";
            handRotationOffset[arm] = torch::tensor(mjcfUtils::stringToArray<float>(v)).permute({1, 2, 3, 0});
        }
    }
}
