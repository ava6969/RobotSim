//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_TABLE_ARENA_H
#define FASTDRL_TABLE_ARENA_H

#include "arena.h"

class TableArena : public Arena {

    torch::Tensor tableFullSize, tableHalfSize, tableFriction, tableOffset, centerPos;
    pugi::xml_node tableBody, tableCollision, tableVisual, tableTop;
    vector<pugi::xml_node> tableLegsVisual;
    bool hasLegs;

public:
    explicit TableArena( torch::Tensor tableFullSize = torch::tensor({0.8, 0.8, 0.05}),
                torch::Tensor tableFriction = torch::tensor({1, 0.005, 0.0001}),
                torch::Tensor tableOffset = torch::tensor({0.f, 0.f, 0.8}),
                bool hasLegs=true,
                const string& xml = "arenas/table_arena.xml");

    void configureLocation();
    torch::Tensor tableTopAbs();

};


#endif //FASTDRL_TABLE_ARENA_H
