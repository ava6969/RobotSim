//
// Created by dewe on 4/14/21.
//

#ifndef FASTDRL_SPACE_H
#define FASTDRL_SPACE_H

#include "torch/torch.h"


class Space
{
public:
    struct Shape
    {
        std::vector<int64_t> dshape{};
        torch::ScalarType dtype{c10::kInt};

        [[nodiscard]] size_t flatten() const {

            if (dshape.empty())
                return 0;
            size_t product=1;
            std::for_each(std::begin(dshape), std::end(dshape), [&product](int64_t dim){ product *= dim;});
            return product;
        }
        void changeDim(std::vector<int64_t> const& v) { dshape = v; }
    };

    template<typename T>
    struct Range
    {
        T low{}, high{};
    };

    protected:
        Shape shape{};

    public:
        Space() = default;
        explicit Space(Shape _shape):shape(std::move(_shape)) {}
        virtual torch::Tensor sample() = 0;
        [[nodiscard]] inline Shape getShape() const { return this->shape; }
        void changeShape( std::vector<int64_t> new_shape) { shape.dshape = std::move(new_shape); }
        virtual ~Space()=default;



};

#endif //FASTDRL_SPACE_H
