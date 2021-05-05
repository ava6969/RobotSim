//
// Created by dewe on 4/27/21.
//

#include "table_arena.h"
#include "../../utils/mjcf_utils.h"

TableArena::TableArena(torch::Tensor _tableFullSize,
                       torch::Tensor _tableFriction,
                       torch::Tensor _tableOffset,
                       bool hasLegs, const string& xml): Arena(mjcfUtils::xmlPathCompletion(xml)),
                                                  tableFullSize(std::move(_tableFullSize)),
                                                  tableHalfSize(tableFullSize/2),
                                                  tableFriction(std::move(_tableFriction)),
                                                  tableOffset(std::move(_tableOffset)), hasLegs(hasLegs)
                                                  {

    centerPos = this->bottomPos + torch::tensor({0, 0, -tableHalfSize[2].item<float>()}) + tableOffset;

    tableLegsVisual = {
            tableBody.first_element_by_path("./geom[@name='table_leg1_visual']"),
            tableBody.first_element_by_path("./geom[@name='table_leg2_visual']"),
            tableBody.first_element_by_path("./geom[@name='table_leg3_visual']"),
            tableBody.first_element_by_path("./geom[@name='table_leg4_visual']")};

    configureLocation();
}

void TableArena::configureLocation() {

    floor.attribute("pos").set_value(mjcfUtils::arrayToString_(bottomPos).c_str());

    tableBody.attribute("pos").set_value(mjcfUtils::arrayToString_(centerPos).c_str());
    tableCollision.attribute("size").set_value(mjcfUtils::arrayToString_(tableHalfSize).c_str());
    tableCollision.attribute("friction").set_value(mjcfUtils::arrayToString_(tableFriction).c_str());
    tableVisual.attribute("size").set_value(mjcfUtils::arrayToString_(tableHalfSize).c_str());
    tableTop.attribute("pos").set_value(mjcfUtils::arrayToString_(
            torch::tensor({0, 0, tableHalfSize[2].item<float>()})).c_str());

    if (!hasLegs)
    {
        for (auto& leg : tableLegsVisual)
        {
            leg.attribute("rgba").set_value("1 0 0 0");
            leg.attribute("size").set_value("0.0001 0.0001");
        }
    }else
    {
        auto delta_x = torch::tensor({0.1, -0.1, -0.1, 0.1});
        auto delta_y = torch::tensor({0.1, 0.1, -0.1, -0.1});
        int i = 0;
        for(auto leg: tableLegsVisual)
        {
            float x = 0;
            auto dx = delta_x[i];
            auto dy = delta_y[i];
            if(tableHalfSize[0].item<float>() > torch::abs(dx * 2.0).item<float>() )
            {
                x += (torch::sign(dx) * tableHalfSize[0] - dx).item<float>();
            }
            float y = 0;
            if(tableHalfSize[1].item<float>() > torch::abs(dy * 2.0).item<float>() )
            {
                y += (torch::sign(dy) * tableHalfSize[1] - dy).item<float>();
            }

            auto z = ((tableOffset[2] - tableHalfSize[2]) / 2.0).item<float>();

            leg.attribute("pos").set_value(mjcfUtils::arrayToString_(torch::tensor({x, y, -z})).c_str() );
            leg.attribute("size").set_value(mjcfUtils::arrayToString_(torch::tensor({0.025, z})).c_str() );
            i++;
        }
    }

}

torch::Tensor TableArena::tableTopAbs()
{
    return torch::tensor(mjcfUtils::stringToArray<float>(floor.attribute("pos").value())) + tableOffset;
}
