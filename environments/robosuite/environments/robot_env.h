//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_ROBOT_ENV_H
#define FASTDRL_ROBOT_ENV_H

#include "base.h"

using Sensor= int;

struct RobotEnvOptions
{
    BaseEnvOptions baseOptions;
    vector<string> robots;
    string env_configuration="default", mount_types="default",  camera_names="agentview", controller_configs{};
    vector<map<string, vector<float>>>initialization_noise{};
    bool use_camera_obs=true, camera_depths=false;
    int camera_heights=256, camera_widths=256;
    vector<map<string, string>>robot_configs{}; //placeholder
};


class RobotEnv : public MujocoEnv {

protected:
    RobotEnvOptions robot_options;

    template<typename T>
    vector<T> input2List(T inp, int length)
    {
        return {inp};
    }

public:

    explicit RobotEnv(RobotEnvOptions const& options);

    void visualize(map<string, bool> const& visSettings) override;

    std::set<string> visualizations() override;

    std::tuple<vector<float>, vector<float>> actionSpec() override;

    vector<int> actionDim() override;

    void loadModel() override;

    void setupReferences() override;

    void setupObservables() override;

    std::tuple<vector<Sensor>, vector<string>>createCameraSensors(string const& camName, int camW,
                                                                  int camH, bool comD, string const& modality);

    void resetInternal() override;

    void preAction(const torch::Tensor &action, bool policyStep = false) override;

    void loadRobots();

    float reward(const torch::Tensor &action) override { return reward(action);}
    bool checkSuccess() override { MujocoEnv::checkSuccess();}

    virtual void checkRobotConfiguration(string const& robots);

    virtual void checkRobotConfiguration(vector<string> const& robots) = 0;

};


template<>
vector<string> RobotEnv::input2List<string>(string inp, int length)
{
    return vector<string>(length, std::move(inp));
}


#endif //FASTDRL_ROBOT_ENV_H
