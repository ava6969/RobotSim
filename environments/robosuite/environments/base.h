//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_BASE_H
#define FASTDRL_BASE_H

#include <functional>
#include "set"
#include "string"
#include "map"
#include "vector"
#include "variant"
#include "../mj_sim.h"
#include "tuple"

using std::string;
using std::vector;
using std::map;
using std::set;

using Observation = std::map<string, vector<torch::Tensor>>;
using Geom = std::variant<string, vector<string>, class MujocoModel*>;
using Preprocesssor = std::function<void(_mjModel*)>;
struct BaseEnvOptions
{
    bool has_renderer= false;
    bool has_offscreen_renderer=true;
    std::string render_camera="frontview";
    bool render_collision_mesh=false;
    bool render_visual_mesh=true;
    int render_gpu_device_id=-1;
    size_t control_freq=20;
    size_t horizon=1000;
    bool ignore_done=false;
    bool hard_reset=true;
};

class MujocoEnv {

protected:
    BaseEnvOptions options;
    Observation observables, obsCache;

    Preprocesssor modelPostProcessor;
    _mjModel* model, mjpyModel;
    _mjData* simStateInitial;
    double timeStep;
    bool done;
    double curTime, modelTimeStep, controlTimestep;
    bool deterministicReset{false};
    std::unique_ptr<MjSim> sim;


protected:

    void postProcessModel();
    virtual void setupReferences();
    virtual void setupObservables();
    void initializeSim (std::string const& xmlString);
    virtual void resetInternal();
    void updateObservables(bool force);
    Observation getObservations(bool forceUpdate=false);
    virtual void preAction(torch::Tensor const& action, bool policyStep=false);
    std::tuple<float, bool, unordered_map<string, string>> postAction(torch::Tensor  const& action);
    void destroyViewer();

public:
    explicit MujocoEnv (BaseEnvOptions options);

    virtual void initializeTime(size_t controlFreq);

    virtual Observation reset();

    virtual std::tuple<Observation, float, bool, map<string, string>> action(vector<float> const& action);

    virtual float reward(torch::Tensor const& action) = 0;

    virtual void render();

    virtual Observation observationSpec();

    virtual void clearObjects(vector<string> objectNames);

    virtual void visualize(map<string, bool> const& visSettings);

    virtual void resetFromXML(string const& xmlString);

    virtual bool checkContext(std::variant<vector<string>, MujocoModel*> geoms1,
                      std::variant< vector<string>, MujocoModel*> geoms2);

    virtual set<string> getContacts(MujocoModel* model);

    virtual void addObservable(class Observable const& observable);

    virtual void modifyObservable(string const& observableName, string const& observableAttribute, std::function<void(void)> modifier);

    virtual bool checkSuccess() = 0;

    virtual std::set<string> observationModalities();

    virtual std::set<string> observationNames();

    virtual std::set<string> enabledObservables();

    virtual std::set<string> activeObservables();

    virtual std::set<string> visualizations();

    virtual std::tuple<vector<float>, vector<float>> actionSpec() = 0;

    virtual vector<int> actionDim() = 0;

    virtual void loadModel();

    void set_model_postprocessor(Preprocesssor const& postprocessor):

    // void _load_model(postprocessor):

//    void close();

//    std::tuple<Observation, float, bool, unordered_map<string, string>> step(const torch::Tensor &action);

};


#endif //FASTDRL_BASE_H
