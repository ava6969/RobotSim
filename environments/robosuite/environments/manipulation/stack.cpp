//
// Created by dewe on 4/24/21.
//

#include "../../models/arenas/table_arena.h"
#include "../../utils/placement_samplers.h"
#include "../../utils/mjcf_utils.h"
#include "stack.h"

void Stack::visualize(const map<string, bool> &visSettings) {
    RobotEnv::visualize(visSettings);

    if(visSettings.at("grippers"))
    {
        visualizeGripperToTarget(robots[0]->gripper, cubeA);
    }
}

bool Stack::checkSuccess() {
    float rStack = 0;
    std::tie(std::ignore, std::ignore, rStack) = stagedReward();
    return rStack > 0;
}

std::tuple<float, float, float> Stack::stagedRewards() {
    auto cubeAPos = sim->data->bodyXPos(cubeABodyID);
    auto cubeBPos = sim->data->bodyXPos(cubeBBodyID);
    auto gripperSitePos = sim->data->site_xpos[robots[0].eefSiteID];
    auto dist = torch::linalg_norm(gripperSitePos - cubeAPos);
    auto rReach = (1 - torch::tanh(10.0 * dist)) * 0.25;

    auto graspingCubeA = checkGrasp(robots[0]->gripper, cubeA);
    if(graspingCubeA)
        rReach += 0.25;

    auto cubeAHeight = cubeAPos[2];
    auto tableHeight = tableOffset[2];
    auto cubeALifted = cubeAHeight > tableHeight + 0.04;
    auto rLift = cubeALifted ? 1.0 : 0.0;

    if(cubeALifted)
    {
        auto horizDist = torch::linalg_norm(cubeAPos.slice(0, 0, 2) - cubeBPos.slice(0, 0, 2));
        rLift += 0.5 * (1 - horizDist.tanh());
    }

    auto rStack = 0;
    auto cubeATouchingCubeB = checkContact(cubeA, cubeB);
    if(!graspingCubeA && rLift > 0 && cubeATouchingCubeB)
        rStack = 2.0;

    return {rReach.item<float>(), rLift.item<float>(), rStack};
}

float Stack::reward(torch::Tensor const& action) {
    auto[rReach, rLift, rStack] = stagedReward();
    float reward = rewardShaping ? std::max<float>(rReach, rLift, rStack) : rStack > 0 ? 2.0 : 0.0;

    if(rewardScale)
    {
        reward *= rewardScale / 2.0;
    }

    return reward;

}

void Stack::loadModel() {
    SingleArmEnv::loadModel();

    auto xPos = robots[0]->robotModel->baseXPosOffset("table")(tableFullSize[0]);
    robots[0]->robotModel.setBaseXPos(xPos);

    mujocoArena = std::make_unique<TableArena>(tableFUllSize, tableFriction, tableOffset);

    mujocoArena->setOrigin({0, 0, 0});

    auto texAttrib = { {"type", "cube"} };

    auto matAttrib = { {"texrepeat", "1 1"},
                  {"specular", "0.4"},
                  {"shininess", "0.1"}};

    CustomMaterial redWood("WoodRed", "redwood", "redwood_mat", texAttrib, matAttrib);
    CustomMaterial greenWood("WoodGreen", "greenwood", "greenwood_mat", texAttrib, matAttrib);

    cubeA = std::make_unique<BoxObject>("cubaA", {0.025, 0.025, 0.025}, {0.025, 0.025, 0.025});
    cubeB = std::make_unique<BoxObject>("cubeB", {0.025, 0.25, 0.025}, {0, 1, 0, 1});

    auto cubes = {cubeA.get(), cubeB,get()};

    if (placementInitializer)
    {
        placementInitializer.reset();
        placementInitializer.add_objects(cubes);
    }else
    {
        placementInitializer = make_unique<UniformRandomSampler>(
                "ObjectSampler",
                cubes,
                {-0.08, 0.08},
                {-0.8, 0.08},
                {},
                false,
                true,
                tableOffset,
                0.01
                );
    }

    std::vector<RobotModel> robots;
    for_each(begin(robots), end(robots),[&](auto& r)
    {
        robots.push_back(r.robotModel);
    })
    model = make_unique<ManipulationTask>(mujocoArena,  ,cubes);

}

void Stack::setupReferences()
{
    RobotEnv::setupReferences();
    cubeABodyID = sim->body_name2id(cubeA.root_body);
    cubeBBodyID = sim->body_name2id(cubeB.root_body);
}

void Stack::resetInternal() {
    RobotEnv::resetInternal();

    if(!deterministicReset)
    {
        auto objectPlacements = placementInitializaer.sample();
        for(auto&[k, v] : objectPlacements)
        {
            auto[objPos, objQuat, obj] = v;
            sim->setJointQPos(obj.joints[0],
                              torch::cat({objPos, objQuat}));
        }
    }
}

void Stack::setupObservables() {
    auto _observables = RobotEnv::setupObservables();

    if(useObjectObs)
    {
        auto pf = robots[0]->robotModel->namingPrefix;
        auto modality = "object";
        using Cache = unordered_map<string, torch::Tensor>;
        std::vector<std::function<torch::Tensor(Cache&)>> sensors = {
                [&] (Cache& obsCache)
                {
                    return sim->bodyXPos(cubeABodyID);
                },
                [&] (Cache& obsCache)
                {
                    return transform::convertQuat(sim->bodyXQuat(cubeABodyID), "xyzw");
                },
                [&](Cache& obsCache)
                {
                    return sim->bodyXPos(cubeBBodyID);
                },
                [&](Cache& obsCache)
                {
                    return transform::convertQuat(sim->bodyXQuat(cubeBBodyID), "xyzw");
                },
                [&](Cache& obsCache)
                {
                    return mjcfUtils::found<std::string>(obsCache, "cubeA_pos") && mjcfUtils::found<std::string>(obsCache, pf+"ee_pos")  ?
                    obsCache["cubeA_pos"] - obsCache[pf + "ee_pos"] : torch::zeros({3});
                },
                [&](Cache& obsCache)
                {
                    return mjcfUtils::found<std::string>(obsCache, "cubeB_pos") && mjcfUtils::found<std::string>(obsCache, pf+"ee_pos")  ?
                           obsCache["cubeABpos"] - obsCache[pf + "ee_pos"] : torch::zeros({3});
                },
                [&](Cache& obsCache)
                {
                    return mjcfUtils::found<std::string>(obsCache, "cubeA_pos") && mjcfUtils::found<std::string>(obsCache, pf+"cubeB_pos")  ?
                           obsCache["cubeA_pos"] - obsCache[pf + "cubeB_pos"] : torch::zeros({3});
                }
        };

        auto names = {"cubeA_pos", "cubeA_quat", "cubeB_pos", "cubeB_quat", "gripper_to_cubeA", "gripper_to_cubeB", "cubeA_to_cubeB"};

        for(int i = 0 ; i < names.size(); i++)
        {
            observables[names[i]] = Observable(names[i], sensors[i], options.control_freq );
        }

        return _observables;

    }

}

Stack::Stack(StackOptions options) : SingleArmEnv(options.base_opt)
{

}
