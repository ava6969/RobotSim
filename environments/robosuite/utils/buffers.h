//
// Created by dewe on 4/30/21.
//

#ifndef FASTDRL_BUFFERS_H
#define FASTDRL_BUFFERS_H

#include "torch/torch.h"

template<class T>
class Buffer
{
    virtual void push(T const& value) = 0;

    virtual void clear() = 0;
};

class DeltaBuffer : public Buffer<torch::Tensor>
{
    int dim;
    torch::Tensor last{torch::zeros(dim)};
    torch::Tensor current{torch::zeros(dim)};
public:

    DeltaBuffer()=default;

    DeltaBuffer(int dim, torch::Tensor initValues={}):dim(dim)
    {
        if(initValues.sizes().size() > 0)
            last = initValues;

    }

    void push(torch::Tensor const& value) override
    {
        last = current;
        current = value;
    }

    void clear() override
    {
        last = torch::zeros(dim);
        current = torch::zeros(dim);
    }

    torch::Tensor delta(bool absValue=false)
    {
        return !absValue ? current  - last : (current - last ).abs();
    }

    torch::Tensor average()
    {
        return (current + last ) / 2.0f;
    }
};

class RingBuffer : public Buffer<torch::Tensor>
{

    void push(torch::Tensor const& value) override
    {

    }

    void clear() override
    {

    }
};

#endif //FASTDRL_BUFFERS_H
