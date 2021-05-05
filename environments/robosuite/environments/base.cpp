//
// Created by dewe on 4/24/21.
//

#include "base.h"
#include "../models/base.h"
#include "../utils/mjcf_utils.h"
#include "../utils/macros.h"

MujocoEnv::MujocoEnv(BaseEnvOptions params): options(std::move(params)) {

    loadModel();

    postProcessModel();

    initializeSim();

    resetInternal();

    observables = setupObservables();
}

void MujocoEnv::postProcessModel() {

    if(modelPostProcessor)
    {
        modelPostProcessor(model);
    }

}


void MujocoEnv::setupReferences()
{

}

void MujocoEnv::setupObservables()
{

}

void MujocoEnv::initializeSim(const std::string& xmlString="")
{
    mjpyModel = mj_loadXML(xmlString.c_str());
    //xmlString.empty() ? std::move(model) :
    sim = std::make_unique<MjSim>(mjpyModel);
    sim->forward();
    initializeTime(options.control_freq);

}

void MujocoEnv::resetInternal()
{
    if(options.has_renderer)
    {
        // study mujoco rendering with opengl simulate.cpp
    }else if(options.has_offscreen_renderer)
    {
        // record.cpp
    }

    simStateInitial = sim->getState();
    setupReferences();
    curTime = 0;
    timeStep = 0;
    done = false;

    obsCache = {};
    for(auto& observable_item : observables)
    {
        observable_item.second.reset();
    }



}

void MujocoEnv::updateObservables(bool force)
{

    for(auto& observable_item : observables)
    {
        observable_item.second.update(modelTimeStep, obsCache, force);
    }
}

Observation MujocoEnv::getObservations(bool forceUpdate)
{
    Observation observations, obsByModality;

    if(forceUpdate)
        updateObservables(true);


    for (auto&[obs_name, observable] : observables)
    {
        if(observable.isEnabled() && observable.isActive())
        {
            auto obs = observable.obs;
            observations[obs_name] = obs;
            auto modality = observable.modality + "-state";
            if(!mjcfUtils::found<vector<float>>(obsByModality, modality))
                obsByModality[modality] = {};
            obsByModality[modality].emplace_back(obs);
        }
    }


#ifdef CONCATENATE_IMAGES
    for (auto&[modality, obs]: obsByModality)
    {
        if(modality == "image-state")
        {
            observations[modality] == torch::cat(obs)
        }
    }
#endif

}

void MujocoEnv::preAction(torch::Tensor  const& action, bool policyStep)
{
    assert(sim->model->nu == action.size(0));
    memcpy(sim->data->ctrl, action.toType(torch::kF64).data_ptr(), sizeof(mjtNum)* sim->model->nu);
}

std::tuple<float, bool, unordered_map<string, string>> MujocoEnv::postAction(torch::Tensor  const& action)
{

    float _reward = reward(action);
    done = (timeStep >= options.horizon) && ! options.ignore_done;

    return {_reward, done, {{}}};
}

void MujocoEnv::initializeTime(size_t controlFreq) {

    curTime = 0;
    modelTimeStep = SIMULATION_TIMESTEP;
    if(modelTimeStep <= 0)
        throw std::runtime_error("invaild simulation timsestep defined");
    this->options.control_freq = controlFreq;
    if(controlFreq <= 0)
        throw std::runtime_error("control freq is invalid");
    controlTimestep = 1 / (double)controlFreq;
}

std::tuple<Observation, float, bool, map<string, string>> MujocoEnv::action(const vector<float> &action)
{
    return std::tuple<Observation, float, bool, map<string, string>>();
}


Observation MujocoEnv::observationSpec() {
    return getObservations();
}

void MujocoEnv::clearObjects(vector<string> objectNames) {

    for(auto& obj : model->mujocoObjects)
    {
        if(mjcfUtils::found(objectNames, obj.name) )
        {
            sim->setJointQPos(obj.joints[0], {10, 10, 10, 1, 0, 0, 0});
        }
    }

}

void MujocoEnv::visualize(map<string, bool> const& visSettings) {

    for(auto& obj : model->mujocoObjects)
    {
        obj->setSItesVisibilit(sim, visSettings["env"]);
    }
}

void MujocoEnv::resetFromXML(const string &xmlString) {

    close();

    deterministicReset = true;

    initializeSim(xmlString);

    reset();

    deterministicReset = false;
}

bool MujocoEnv::checkContext(std::variant<vector<string>, MujocoModel *> geoms1,
                             std::variant<vector<string>, MujocoModel *> geoms2) {

    vector<string> _geom1 , _geom2;
    auto g1 =  std::get<MujocoModel*>(geoms1);
    auto g2 =  std::get<MujocoModel*>(geoms2);
    vector<string> _geom1 = g1 ? g1->contactGeoms() :  std::get<vector<string>>(geoms1);
    vector<string> _geom2 = g2 ? g2->contactGeoms() :  std::get<vector<string>>(geoms2);

    for(vector<mjContact> & contact :  sim->data->getContact())
    {


    }

    return false;
}

set<string> MujocoEnv::getContacts(MujocoModel *_model)
{
    set<string> contactSet;
    for(auto& contact: sim->data->getContact())
    {

    }
    return contactSet;
}

void MujocoEnv::addObservable(const Observable &observable) {

    assert(mjcfUtils::found(observables, observable.name));
    observables[observable.name] = observable;
}

void MujocoEnv::modifyObservable(const string &observableName, const string &observableAttribute,
                                 std::function<void(void)> modifier) {

    assert(mjcfUtils::found(observables, observable.name));
    auto obs = observables[observableName];

    if(observableAttribute == "sensor")
    {
        obs.setSensor(modifier);
    }
    else if(observableAttribute == "corrupter")
    {
        obs.setCorrupter(modifier);
    }
    else if(observableAttribute == "filter")
    {
        obs.setFilter(modifier);
    }
    else if(observableAttribute == "delayer")
    {
        obs.setDelayer(modifier);
    }
    else if(observableAttribute == "sampling_rate")
    {
        obs.setSamplingRate(modifier);
    }
    else if(observableAttribute == "enabled")
    {
        obs.setEnabled(modifier);
    }
    else if(observableAttribute == "active")
    {
        obs.setActive(modifier);
    }else
    {
        throw ("invalid attribute");
    }

}

std::set<string> MujocoEnv::observationModalities() {

    std::set<string> modalities;
    std::transform(begin(observables), end(observables), back_inserter(modalities), [](auto& item)
    {
        return item.second;
    });

    return modalities;
}

std::set<string> MujocoEnv::observationNames() {

    std::set<string> names;
    std::transform(begin(observables), end(observables), back_inserter(names), [](auto& item)
    {
        return item.first;
    });

    return names;

}

std::set<string> MujocoEnv::visualizations() {
    return {"env"};
}

std::set<string> MujocoEnv::activeObservables() {
    std::set<string> names;
    std::for_each(begin(observables), end(observables), [&names](auto& item)
    {
        if(item.second.isActive())
            names.insert(item.first);
    });

    return names;
}

std::set<string> MujocoEnv::enabledObservables() {
    std::set<string> names;
    std::for_each(begin(observables), end(observables), [&names](auto& item)
    {
        if(item.second.isEnabled())
            names.insert(item.first);
    });

    return names;
}

void MujocoEnv::render() {

    // render
}

Observation MujocoEnv::reset()
{
    if(options.hard_reset && not deterministicReset)
    {
        destroyViewer();
        loadModel();
        postProcessModel();
        initializeSim();
    }else
        sim->reset();

    resetInternal();
    sim->forward();
    obsCache = {};

    if(options.hard_reset )
    {
        observables = setupObservables();
        for(auto const& item : observables)
        {
            auto& name = item.first;
            auto& obs = item.second;
            modifyObservable(name, "sensor", obs._sensor);
        }
    }

    map<string, bool> visSettings;
    std::transform(begin(visualizations()), end(visualizations()), back_inserter(visSettings),[](auto& viz){
       return std::make_pair<string, bool>(viz, false);
    });
    visualize(visSettings);

    return getObservations(true);
}

void MujocoEnv::set_model_postprocessor(Preprocesssor const& postprocessor) {

    modelPostProcessor = postprocessor;
}

void MujocoEnv::loadModel() {

}
//
//std::tuple<Observation, float, bool, unordered_map<string, string>> MujocoEnv::step(torch::Tensor const& action)
//{
//
//    if(done)
//        throw("executing action in terminated episode");
//
//    timeStep += 1;
//
////    bool policyStep = true;
////
////    for(int i =0; i < int(controlTimestep / modelTimeStep); i++)
////    {
////        sim->forward();
////        preAction(action, policyStep);
////        sim->step();
////        updateObservables();
////        policyStep = false;
////    }
////    curTime += controlTimestep;
////
////    auto[reward, done, info] = postAction(action);
////    return {getObservations(), reward. done, info};
//
//    return {};
//}

//void MujocoEnv::close()
//{
//    destroyViewer();
//}

void MujocoEnv::destroyViewer()
{

//    if(viewer)
//    {
//        viewer.close();
//        viewer = nullptr;
//    }
}
