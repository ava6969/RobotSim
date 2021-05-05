//
// Created by dewe on 4/24/21.
//

#include "robot_env.h"
#include "../utils/globals.h"
#include "../utils/macros.h"
RobotEnv::RobotEnv(const RobotEnvOptions &options): MujocoEnv(options.baseOptions), robot_options(options)
{
    checkRobotConfiguration(options.robots);
    numRobots = options.robots.size();
    robotNames = options.robots;
    robots = input2List(std::string(), 0);

    auto mountTypes = input2List(std::string(), 0);

    auto controllerConfigs = input2List(std::string(), 0);

    auto initializationNoise = input2List(std::string(), 0);

    numCameras = options.camera_names.size();

    cameraHeights = input2List(std::string(), 0);
    cameraWidths = input2List(std::string(), 0);
    cameraDepths = input2List(std::string(), 0);

    if(options.use_camera_obs && ! options.baseOptions.has_offscreen_renderer)
        throw std::runtime_error("Error: Camera observations require an offscreen renderer!");
    if(options.use_camera_obs && ! options.camera_names.empty())
        throw std::runtime_error("Must specify at least one camera name when using camera obs");

    if(robot_options.robot_configs.empty())
    {
        robot_options.robot_configs.resize(numRobots);
    }


//    vector<unordered_map<string, >>
//    auto map_ = {
//            {"controller_config", controllerConfigs[]},
//            {},
//            {},
//            {}
//    };
//    robot_options.robot_configs

}

void RobotEnv::visualize(const map<string, bool> &visSettings) {
    MujocoEnv::visualize(visSettings);

    for(auto& robot: robots)
    {
        robots.visualize(visSettings);
    }
}

std::set<string> RobotEnv::visualizations() {
    auto visSet = MujocoEnv::visualizations();
    visSet.insert("robots");
    return visSet;
}

std::tuple<vector<float>, vector<float>> RobotEnv::actionSpec() {

    vector<float> low, high;

    for(auto& robot: robots)
    {
        auto[lo, hi] = robot.actionLimits();
        low.insert(end(low), begin(lo), end(lo));
        high.insert(end(high), begin(hi), end(hi));
    }

    return {low, high};
}

vector<int> RobotEnv::actionDim() {
    return actionDim;
}

void RobotEnv::loadModel() {
    MujocoEnv::loadModel();
    loadRobots();
}

void RobotEnv::setupReferences() {
    MujocoEnv::setupReferences();

    for(auto& robot: robots)
    {
        robot.resetSim(sim.get());
        robot.setupReferences();
    }

}

void RobotEnv::setupObservables() {
    auto observables = MujocoEnv::setupObservables();
    for(auto const& robot: robots)
    {
        observables.update(robot.SetObservables());
    }

    if(robot_options.use_camera_obs)
    {
        vector<string> sensors, names;
        for(int i = 0; i < cameraNames.size(); i++)
        {
            auto camName = cameraNames[i];
            auto camW = cameraWidth[i];
            auto camH = cameraHeight[i];
            auto camD = cameraDepth[i];

            auto[camSensors, camSensorsNames] = createCameraSensors(camName camW, camH, camD, "image");
            sensors.push_back(camSensors);
            names.push_back(camSensorsNames);
        }

        for(int i = 0; i < names.size(); i++)
        {
            observables[names[i]] = Observable(names[i], sensors[i], robot_options.baseOptions.control_freq);

        }

    }

    return observables;

}


std::tuple<vector<Sensor>, vector<string>> RobotEnv::createCameraSensors(string const& camName, int camW,
                                                                         int camH, bool camD, string const& modality)
{
    auto convention = globs::IMAGE_CONVENTION_MAPPING.at(IMAGE_CONVENTION);

    vector<Sensor> sensors;
    vector<string> names;

    auto rgbSensorName = camName + "_image";
    auto depthSensorName = camName + "_depth";

    sensors.push_back(cameraRGB);
    names.push_back(rgbSensorName);

    if(camD)
    {
        sensors.push_back(cameraDepth);
        names.push_back(depthSensorName);
    }


    return {sensors, names};
}

void RobotEnv::resetInternal() {
    MujocoEnv::resetInternal();

    resetControllers();

    actionDim = 0;

    for(auto& robot: robots)
    {
        robot.reset(deterministicReset);
        actionDim += robot.actionDim()
    }

    if(robot_options.use_camera_obs)
    {
        for(auto camName: robot_options.camera_names)
        {
//            camName.start
        }
        robot_options.camera_names = tempNames;
    }


}

void RobotEnv::preAction(const torch::Tensor &action, bool policyStep) {

    assert(action.size(0) == actionDim);

    int cutOff = 0;
    for(int idx = 0; idx < robot_options.robots.size(); idx++)
    {
        auto robotAction = action.slice(0, cutOff, cutOff + robots[idx].actionDim);
        robots[idx].control(robotAction, policyStep);
        cutOff += robots[idx].actionDim;
    }
}

void RobotEnv::loadRobots() {

    for(int i = 0; i < robotNames.size()l i++)
    {
        robots[i] = createRobot()
        robots[i].load_model();
    }

}

void RobotEnv::checkRobotConfiguration(const string &robots)
{

    checkRobotConfiguration(vector<string>{robots});
}

