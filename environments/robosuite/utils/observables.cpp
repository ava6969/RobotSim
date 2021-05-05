//
// Created by dewe on 4/28/21.
//

#include "observables.h"

Observable::Observable(ObservableOptions option):
name(option.name),
sensor(std::move(option.sensor)),
corrupter(std::move(option.corrupter)),
filter(std::move(option.filter)),
delayer(std::move(option.delayer)),
sampling_time_step(1.f / option.sampling_rate),
enabled(option.enabled),
active(option.active),
isNumber(false),
dataShape({1, }),
sampled(false)

{
    checkSensorValidity();
    reset();
}

void Observable::reset() {

    timeSinceLastSample  = 0.0;
    currentDelay = delayer->get();
    currentObservedValue = torch::zeros(dataShape);

}

void Observable::update(float timeStep, unordered_map<string, torch::Tensor> obsCache, bool force) {

    auto Sample = [&]() mutable
    {
        auto _obs = filter->get(corrupter->get(sensor->get(obsCache)));
        currentObservedValue = (_obs.sizes().size() == 1 && _obs.size(0) == 1 ) ? _obs[0] : _obs;
        obsCache[name] = currentObservedValue;
        currentDelay = delayer->get();
    };
    if(enabled)
    {
        timeSinceLastSample += timeStep;

        if((not sampled && sampling_time_step - currentDelay >= timeSinceLastSample) || force)
        {
            Sample();
            sampled = true;
        }
        if(timeSinceLastSample >= sampling_time_step)
        {
            if(! sampled)
            {
                printf("Warning: sampling rate for observable %s is either too low or delay is too high. "
                       "Please adjust one (or both)", name.c_str());

                Sample();
            }
            timeSinceLastSample =  int(timeSinceLastSample) % int(sampling_time_step);
            sampled = false;
        }

    }

}

void Observable::checkSensorValidity()
{
    try {
        auto mod = modality();
        dataShape = sensor->get({{}}).sizes().vec();
        isNumber = dataShape.size() == 1 && dataShape[0] == 1;
    } catch (std::exception const& exp) {

        throw std::runtime_error("Current sensor for observable "+name+" is invalid.");
    }
}
