//
// Created by dewe on 4/26/21.
//

#pragma once


#include "string"
#include "numeric"
#include "map"
#include "torch/torch.h"

using namespace torch::indexing;


namespace transform
{

    torch::Tensor  unit_vector(torch::Tensor data,
                               std::optional<int64_t> axis=std::nullopt,
                               std::optional<torch::Tensor> out=std::nullopt);

    torch::Tensor quatSlerp(torch::Tensor const& quat0, torch::Tensor  const& quat1, double fraction, bool shortestpath=true);

    torch::Tensor mat2quat(torch::Tensor const& rMat);

    torch::Tensor euler2mat( torch::Tensor const& euler);

    torch::Tensor mat2euler(torch::Tensor const& rMat, std::string const& axes="sxyz");

    torch::Tensor quat2mat(torch::Tensor const& quat);

    torch::Tensor convertQuat(torch::Tensor const& q , std::string const& to="xyzw");

    torch::Tensor quatMultiply(torch::Tensor const& q0 , torch::Tensor const& q1);

    torch::Tensor quatConjugate(torch::Tensor const& q0);

    torch::Tensor quatInverse(torch::Tensor const& q0);

    torch::Tensor quatDistance(torch::Tensor const& q0, torch::Tensor const& q1);

    std::array<float, 2>  randomAxisAngle(float* angleLimit);

    torch::Tensor randomQUat(torch::Tensor rand);

    template<typename T>
    torch::Tensor vec(std::vector<T> const& values)
    {
            return torch::tensor(values);
    }

    template<typename T>
    torch::Tensor mat4(std::vector<T> const& values)
    {
    return torch::tensor(values).view({4, 4});
    }

    torch::Tensor makePose(torch::Tensor const& translation, torch::Tensor const& rotation)
    {
        auto pose = torch::zeros({4, 4});
        pose.index_put_({Slice(None, 3), Slice(None, 3)}, rotation);
        pose.index_put_({Slice(None, 3), 3}, translation);
        pose[3][3] = 1.f;
        return pose;
    }

    torch::Tensor poseInv(torch::Tensor const& pose)
    {
        auto pose_inv = torch::zeros({4, 4});
        auto p = pose.index({Slice(None, 3), Slice(None, 3)});
        pose_inv.index_put_({Slice(None, 3), Slice(None, 3)}, p.t());
        auto res = -p.t().dot(pose.index({Slice(None, 3), 3}));
        pose_inv.index_put_({Slice(None, 3), 3}, res);
        pose_inv[3][3] = 1.f;
        return pose_inv;
    }

    torch::Tensor pose_in_A_to_pose_in_B(torch::Tensor const& poseA, torch::Tensor const& pose_A_in_B)
    {
        return pose_A_in_B.dot(poseA);
    }


};
