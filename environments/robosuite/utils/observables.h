//
// Created by dewe on 4/28/21.
//

#ifndef FASTDRL_OBSERVABLES_H
#define FASTDRL_OBSERVABLES_H

#include "sensors.h"


struct Corrupter
{
    float low={std::numeric_limits<float>::min() }, high{std::numeric_limits<float>::max() };
    virtual torch::Tensor get(torch::Tensor const& inp)
    {
        return inp;
    }

};

struct DeterministicCorrupter : public Corrupter
{
    float corruption;

    torch::Tensor get(torch::Tensor const& inp) override
    {
        return (inp + corruption).clip(low, high);
    }
};


struct UniformNoiseCorrupter : public Corrupter
{
    float minNoise, maxNoise;

    torch::Tensor get(torch::Tensor const& inp) override
    {
        auto noise = (maxNoise-minNoise) * torch::rand_like(inp) + minNoise;
        return (inp + noise).clip( low, high);
    }
};

struct GaussianNoiseCorrupter : public Corrupter
{
    float mean, std;

    torch::Tensor get(torch::Tensor const& inp) override
    {
        auto noise = mean + std* torch::rand_like(inp);
        return (inp + noise).clip( low, high);
    }
};

struct Delayer
{
    virtual float get()
    {
        return 0.f;
    }
};

struct DeterministicDelayer : Delayer
{
    float delay;
    float get() override
    {
        assert(delay >= 0 );
        return delay;
    }
};

struct UniformSampledDelayer : Delayer
{
    float minDelay, maxDelay;

    float get() override
    {
        assert(std::min<float>(minDelay, maxDelay) >= 0);
        return (minDelay + (maxDelay - minDelay) * torch::rand({1})).item<float>();
    }
};

struct GaussianSampledDelay : Delayer
{
    float mean, std;

    float get() override
    {
        assert(mean >= 0);
        return std::max<float>(0.f, (mean+std * torch::randn({1})).round().item<int>());
    }
};

struct ObservableOptions
{
    string name;
    std::unique_ptr<Sensor> sensor;
    std::unique_ptr<Corrupter> corrupter;
    std::unique_ptr<Corrupter> filter;
    std::unique_ptr<Delayer> delayer;
    float sampling_rate=20;
    bool enabled=true;
    bool active=true;
};

class Observable {

protected:
    string name;
    std::unique_ptr<Sensor> sensor;
    std::unique_ptr<Corrupter> corrupter;
    std::unique_ptr<Corrupter> filter;
    std::unique_ptr<Delayer> delayer;
    float sampling_time_step;
    bool enabled=true;
    bool active=true;
    bool isNumber = false;
    std::vector<int64_t> dataShape;
    float currentDelay;
    float timeSinceLastSample = 0.0;
    torch::Tensor currentObservedValue;
    bool sampled;



public:
    explicit Observable(ObservableOptions option);

    void update(float timeStep, unordered_map<string, torch::Tensor> obsCache, bool force=false);

    void reset();

    bool isEnabled() { return enabled; }

    bool isActive() { return active; }

    void setEnabled(bool  _enabled) { enabled = _enabled; reset();}

    void setActive(bool _active) { active = _active; }

    void setSensor(Sensor* _sensor) { *sensor = *_sensor;}

    void setCorrupter(Corrupter* corrupter1) { *corrupter = *corrupter1; }

    void setFilter(Corrupter* filter1) { *filter = *filter1; }

    void setDelayer(Delayer* delayer1) { *delayer = *delayer1; }

    void setSamplingRate(int rate) { assert(rate>0); sampling_time_step = 1/rate; }

    void checkSensorValidity();

    torch::Tensor obs() { return active ? currentObservedValue : torch::tensor({}); }

    inline string modality() const { return sensor->modality; }




};


#endif //FASTDRL_OBSERVABLES_H
