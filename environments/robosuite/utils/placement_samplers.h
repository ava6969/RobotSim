//
// Created by dewe on 4/28/21.
//

#ifndef FASTDRL_PLACEMENT_SAMPLERS_H
#define FASTDRL_PLACEMENT_SAMPLERS_H

#include "torch/torch.h"
#include "string"
#include "vector"
#include "array"
#include "variant"

using std::string;
using std::array;

class ObjectPositionSampler {


protected:
    struct Fixture
    {
        struct Pos{
            float x, y, z;
        } pos{};

        struct Quat
        {
            float x, y, z, w;
        }quat{};

        class MujocoObject* otherObj{};
    };

    struct BaseSamplerOptions
    {
        std::string const& name;
        bool ensureObjectBoundaryInRange{};
        bool ensureValidPlacement{};
        torch::Tensor const& referencePos;
        float zOffset;
    };

    string name;
    std::vector<class MujocoObject* > mujocoObjects;
    bool ensure_object_boundary_in_range;
    bool ensure_valid_placement;
    torch::Tensor reference_pos;
    float zOffset;


public:
    ObjectPositionSampler(BaseSamplerOptions const& options, std::vector<class MujocoObject*> const& mujocoObjects);

    void addObjects(class MujocoObject* mujocoObject);
    void addObjects(std::vector<class MujocoObject*> const& mujocoObjects);

    void reset();

    virtual std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture> fixtures,
                        torch::Tensor const& reference,
                        bool onTop)=0;

    virtual std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture> fixtures,
                        std::string const& reference,
                        bool onTop)=0;

    virtual std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture> fixtures, bool onTop)=0;
};

class UniformRandomSampler: public ObjectPositionSampler
{

    struct UniformSamplerOptions {

        BaseSamplerOptions baseOpt;
        std::array<float, 2> xRange{}, yRange{};
        char rotationAxis='z';
    };

    std::optional< std::variant<float, array<float, 2> > > rotation;
    std::array<float, 2> xRange{}, yRange{};
    char rotationAxis='z';

public:

    explicit UniformRandomSampler(UniformSamplerOptions const& option,
                                  std::vector<MujocoObject *> const& mujocoObjects,
                                  std::variant<float, array<float, 2> >  const& rotation);

    float sampleX(float objectHorizontalRadius);

    float sampleY(float objectHorizontalRadius);

    torch::Tensor sampleQuat();

    std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture>fixtures,
                torch::Tensor const& reference,
                        bool onTop) override;

    std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture>fixtures,
                        std::string const& reference,
                        bool onTop) override;

    std::unordered_map<string, Fixture> sample(std::unordered_map<string, Fixture> fixtures, bool onTop) override;

};


#endif //FASTDRL_PLACEMENT_SAMPLERS_H
