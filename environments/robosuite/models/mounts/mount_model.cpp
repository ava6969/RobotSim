//
// Created by dewe on 4/27/21.
//

#include "mount_model.h"
#include "../../utils/globals.h"
#include "../../utils/mjcf_utils.h"
std::array<float, 4> MountModel::contactGeomRGBA()
{
    return globs::MOUNT_COLLISION_COLOR;
}

MountModel::MountModel(string fName, int idn): MujocoXMLModel(fName, idn) {

    auto attr = worldBody.attribute("quat");
    auto vec_str = attr ? attr.value() : "1 0 0 0";

    auto vec = mjcfUtils::stringToArray<double>(vec_str);
    rotationOffset = torch::tensor(vec).permute({1, 2, 3, 0});

}
