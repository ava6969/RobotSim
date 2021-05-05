//
// Created by dewe on 4/27/21.
//

#include "arena.h"
#include "../../utils/mjcf_utils.h"
#include "../../utils/globals.h"

Arena::Arena(string fName) : MujocoXML(std::move(fName)) {

    bottomPos = torch::zeros({3});
    floor = worldBody.first_element_by_path("./geom[@name='floor']");

    mjcfUtils::recolorCollisionGeoms(worldBody, globs::ENVIRONMENT_COLLISION_COLOR,
                                     [](auto e)
                                     {
                                        return e.attribute("name").value() == "floor";
                                     });

}

void Arena::setOrigin(const torch::Tensor& offset)
{
    mjcfUtils::GetAllPredicate walker([](pugi::xml_node n)
                                      {
                                          return strcmp(n.name(), "pos") == 0;
                                      }, false);
    worldBody.traverse(walker);
    for(auto& node: walker.nodes)
    {
        auto curPos = mjcfUtils::stringToArray<float>(node.attribute("pos").value());
        auto newPos = torch::tensor(curPos) + offset;
        node.attribute("pos").set_value(mjcfUtils::arrayToString_(newPos).c_str());
    }

}

void Arena::setCamera(string cameraName, torch::Tensor pos, torch::Tensor quat,
                      std::unordered_map<string, string> cameraAttribs) {

    auto camera = mjcfUtils::findElements(worldBody, "camera", {{"name", cameraName}}, true);

    cameraAttribs["pos"] = mjcfUtils::arrayToString_(pos);
    cameraAttribs["quat"] = mjcfUtils::arrayToString_(quat);

    if (!camera)
    {
        worldBody.append_move(mjcfUtils::newElement("camera", cameraName, cameraAttribs));
    }else
    {
        for(auto& item: cameraAttribs)
        {
            camera.attribute(item.first.c_str()).set_value(item.second.c_str());
        }
    }


}
