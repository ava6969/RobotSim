//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_ARENA_H
#define FASTDRL_ARENA_H

#include "../base.h"
#include "string"
#include "torch/torch.h"

using std::string;

class Arena : public MujocoXML {

protected:
    pugi::xml_node floor;
    torch::Tensor bottomPos;
public:
    explicit Arena(string fName);

    void setOrigin(const torch::Tensor& offset);

    void setCamera(string cameraName,
                   torch::Tensor pos,
                   torch::Tensor quat,
                   std::unordered_map<string, string> cameraAttribs={});

};


#endif //FASTDRL_ARENA_H
