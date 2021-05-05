//
// Created by dewe on 4/28/21.
//

#include "placement_samplers.h"
#include "../models/objects/objects.h"
#include "mjcf_utils.h"

ObjectPositionSampler::ObjectPositionSampler(const ObjectPositionSampler::BaseSamplerOptions &options,
                                             std::vector<MujocoObject *> const& mujocoObjects) : name(options.name),
                                             mujocoObjects(mujocoObjects),
                                             ensure_object_boundary_in_range(options.ensureObjectBoundaryInRange),
                                             ensure_valid_placement(options.ensureValidPlacement),
                                             zOffset(options.zOffset)
                                             {



}


void ObjectPositionSampler::addObjects(std::vector<MujocoObject *> const& _mujocoObjects) {

    for(auto& obj: mujocoObjects)
    {
        assert(not mjcfUtils::found(_mujocoObjects, obj));
        mujocoObjects.push_back(obj);
    }

}

void ObjectPositionSampler::addObjects(struct MujocoObject *mujocoObject) {

    addObjects(std::vector<MujocoObject *>{mujocoObject});
}

void ObjectPositionSampler::reset() {

    mujocoObjects.clear();
}

UniformRandomSampler::UniformRandomSampler(const UniformRandomSampler::UniformSamplerOptions &option,
                                           std::vector<MujocoObject *> const& mujocoObjects,
                                           const std::variant<float, array<float, 2>> &rotation):
                                           ObjectPositionSampler(option.baseOpt, mujocoObjects),
                                           xRange(option.xRange),
                                           yRange(option.yRange),
                                           rotationAxis(option.rotationAxis)
                                           {
}

float UniformRandomSampler::sampleX(float objectHorizontalRadius) {
    if(ensure_object_boundary_in_range)
    {
        xRange[0] += objectHorizontalRadius;
        xRange[1] -= objectHorizontalRadius;
    }
    return torch::rand({1}).uniform_(xRange[0], xRange[1]).item<float>();
}

float UniformRandomSampler::sampleY(float objectHorizontalRadius) {
    if(ensure_object_boundary_in_range)
    {
        yRange[0] += objectHorizontalRadius;
        yRange[1] -= objectHorizontalRadius;
    }
    return torch::rand({1}).uniform_(xRange[0], xRange[1]).item<float>();
}

torch::Tensor UniformRandomSampler::sampleQuat()
{
    float rotAngle;
    if(!this->rotation.has_value())
    {
        rotAngle = torch::rand({1}).uniform_(2*M_PI, 0).item<float>();
    }else if(std::holds_alternative<array<float, 2>>(rotation.value()))
    {
        auto bound = std::get<array<float, 2>>(rotation.value());
        rotAngle = torch::rand({1}).uniform_(bound[0], bound[1]).item<float>();
    }else
        rotAngle = std::get<float>(rotation.value());

    switch(rotationAxis)
    {
        case 'x':
            return torch::tensor({cos(rotAngle/2), sin(rotAngle/2), 0, 0});
        case 'y':
            return torch::tensor({cos(rotAngle/2), 0, sin(rotAngle/2), 0});
        case 'z':
            return torch::tensor({cos(rotAngle/2), 0, 0, sin(rotAngle/2)});

        default:
            throw std::runtime_error("Invalid rotation axis specified. Must be 'x', 'y', or 'z'");

    }

    return torch::Tensor();
}

unordered_map<string, ObjectPositionSampler::Fixture> UniformRandomSampler::sample(unordered_map<string, Fixture> placedObjects, const torch::Tensor &baseOffset,
                                  bool onTop) {

    assert(baseOffset.size(0) == 3);

    for(auto & obj: mujocoObjects)
    {
        assert(!found<std::string>(placedObjects, obj->name()));

        auto horizontalRadius = obj->horizontalRadius();
        auto bottomOffset = obj->bottomOffset();
        bool success = false;
        for(int i = 0; i < 5000; i++)
        {
            auto objX = (sampleX(horizontalRadius) + baseOffset[0]).item<float>();
            auto objY = (sampleY(horizontalRadius) + baseOffset[1]).item<float>();
            auto objZ = (zOffset + baseOffset[2]).item<float>();

            if(onTop)
                objZ -= bottomOffset[-1].item<float>();

            bool locationValid = true;
            if(ensure_valid_placement)
            {
                for(auto const& item : placedObjects)
                {
                    auto fixture = item.second;
                    if(torch::norm(torch::tensor({objX - fixture.pos.x, objY - fixture.pos.y})).item<float>() <=
                            (fixture.otherObj->horizontalRadius() + horizontalRadius)  &&
                            (objZ - fixture.pos.z) <= (fixture.otherObj->topOffset()[-1] - bottomOffset[-1]).item<float>()
                    )
                    {
                        locationValid = false;
                        break;
                    }
                }
            }

            if(locationValid)
            {
                auto quat = sampleQuat();
                // what has initQUat as an attribute


                placedObjects[obj->name()] = {objX, objY, objZ};
                success = true;
                break;
            }
        }

        if(not success)
            throw std::runtime_error("cannot place all objects");
    }

    return placedObjects;

}

unordered_map<string, ObjectPositionSampler::Fixture> UniformRandomSampler::sample(unordered_map<string, Fixture> fixtures, bool onTop) {
    return sample(fixtures, reference_pos, onTop);
}

unordered_map<string, ObjectPositionSampler::Fixture>
UniformRandomSampler::sample(unordered_map<string, Fixture> fixtures, const string &reference, bool onTop) {

    assert(found<string>(fixtures, reference));

    auto fix = fixtures[reference];
    auto baseOff = torch::tensor({fix.pos.x, fix.pos.y, fix.pos.z });
    if(onTop)
        baseOff += torch::tensor({0.f, 0.f, fix.otherObj->topOffset()[-1].item<float>()});

    return sample(fixtures, baseOff, onTop);

}
