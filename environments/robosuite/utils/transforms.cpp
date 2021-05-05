//
// Created by dewe on 4/27/21.
//

//
// Created by dewe on 4/26/21.
//

#include "transforms.h"
#include "globals.h"

using std::map;

namespace transform
{

    torch::Tensor convertQuat(torch::Tensor const& q , string const& to)
    {
        if(to == "xyzw")
            return q.index({{1, 2, 3, 0}});
        if(to == "wxyz")
            return q.index({{3, 0, 1, 2}});

        throw std::runtime_error("convert quat: choose a valid atg(xyzw or wxyz");
    }

    torch::Tensor quatMultiply(torch::Tensor const& q0 , torch::Tensor const& q1)
    {
        float x0, y0, z0, w0, x1, y1, z1, w1;

        x0 = q0[0].item<float>();
        y0 = q0[1].item<float>();
        z0 = q0[2].item<float>();
        w0 = q0[3].item<float>();

        x1 = q1[0].item<float>();
        y1 = q1[1].item<float>();
        z1 = q1[2].item<float>();
        w1 = q1[3].item<float>();

        return vec<float>({
                           x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                           -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                           x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                           -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        });
    }

    torch::Tensor quatConjugate(torch::Tensor const& quaternion)
    {
        return vec<float>( {-quaternion[0].item<float>(), -quaternion[1].item<float>(), -quaternion[2].item<float>(), quaternion[3].item<float>()});
    }

    torch::Tensor quatInverse(torch::Tensor const& quaternion)
    {
        return quatConjugate(quaternion) / torch::dot(quaternion, quaternion);
    }

    torch::Tensor quatDistance(torch::Tensor const& q0, torch::Tensor const& q1)
    {
        return quatMultiply(q1, quatInverse(q0));
    }

    std::array<float, 2> randomAxisAngle(const float* angleLimit)
    {
        float lim = 2 * M_PI;
       if (!angleLimit)
           angleLimit = &lim;

       auto randomAxis  = torch::randn(3);
       randomAxis /= torch::norm(randomAxis);
       auto randomAngle = torch::rand({1}).uniform_(0, *angleLimit);

       return {randomAxis.item<float>(), randomAngle.item<float>()};

    }

    torch::Tensor randomQUat(torch::Tensor rand)
    {
        if(rand.sizes().empty())
        {
            rand = torch::rand({3});
        }else
        {
            assert(rand.size(0) == 3 && rand.sizes().size() == 1);
        }

        auto r1 = torch::sqrt(1.0 - rand[0]);
        auto r2 = torch::sqrt(rand[0]);
        auto pi2 = M_PI * 2.0;
        auto t1 = pi2 * rand[1];
        auto t2 = pi2 * rand[2];

        return vec<float>({
                   (t1.sin() * r1).item<float>(),
                   (t1.cos() * r1).item<float>(),
                   (t2.cos() * r2).item<float>()});
    }

    torch::Tensor  unit_vector(torch::Tensor data,
                               std::optional<int64_t> axis,
                               std::optional<torch::Tensor> out)
    {
        if(!out.has_value())
        {
            if (data.sizes().size() == 1)
            {
                data /= data.dot(data).sqrt();
                return data;
            }
        }else
        {
            if(out.value().ne_(data).sum().item<float>() != 0.f)
            {
                out = data;
            }
            data = out.value();
        }
        auto length = torch::atleast_1d(torch::sum(data*data, axis.value())).sqrt();

        if(axis.has_value())
            length = length.unsqueeze(axis.value());

        data /= length;

        if(!out.has_value())
            return data;

        return {};
    }

    torch::Tensor quatSlerp(torch::Tensor const& quat0, torch::Tensor  const& quat1, double fraction, bool shortestpath)
    {

        torch::Tensor q0 = unit_vector(quat0);
        torch::Tensor q1 = unit_vector(quat1);

        if (fraction == 0.0)
            return q0;
        else if (fraction == 1.0)
            return q1;

        torch::Tensor d = q0.dot(q1);

        if ((d.abs() - 1.0).abs().item<float>() < globs::EPS)
            return q0;

        if (shortestpath && d.item<float>() < 0.0)
        {
            // invert rotation
            d = -d;
            q1 *= -1.0;
        }

        torch::Tensor angle = d.clip( -1, 1).acos();
        if (angle.abs().item<float>() < globs::EPS)
            return q0;

        torch::Tensor isin = 1.0 / angle.sin();

        q0 *= ((1.0 - fraction) * angle).sin() * isin;
        q1 *= (fraction * angle).sin() * isin;
        q0 += q1;
        return q0;

    }

    torch::Tensor mat2quat(torch::Tensor const& rMat)
    {
        auto M = rMat.index({torch::indexing::Slice(0, 3),
                             torch::indexing::Slice(0, 3)});

        auto m00 = M[0][0].item<float>();
        auto m01 = M[0][0].item<float>();
        auto m02 = M[0][0].item<float>();
        auto m10 = M[1][0].item<float>();
        auto m11 = M[1][1].item<float>();
        auto m12 = M[1][2].item<float>();
        auto m20 = M[2][0].item<float>();
        auto m21 = M[2][1].item<float>();
        auto m22 = M[2][2].item<float>();

        auto K = torch::stack(vector<torch::Tensor>{
                torch::tensor({m00 - m11 - m22, 0.f, 0.f, 0.f}),
                torch::tensor({m01 + m10, m11 - m00 - m22, 0.f, 0.0f}),
                torch::tensor({m02 + m20, m12 + m21, m22 - m00 - m11, 0.f}),
                torch::tensor({m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22})});
        K = K.view({3, 3}) / 3.0;
        auto[w, V] = torch::linalg_eigh(K);
        auto inds = torch::tensor({3, 0, 1, 2});
        auto q1 = V.index({inds, V.argmax()});
        if(q1[0].item<float>() < 0.f)
            torch::negative_out(q1, q1);
        inds = torch::tensor({1, 2, 3, 0});

        return q1.index({inds});
    }

    torch::Tensor euler2mat( torch::Tensor const& euler)
    {
        torch::Tensor res = torch::empty({3, 3});
        assert(euler.size(-1) == 3);

        auto ai = -euler.index({"...", 2});
        auto aj = -euler.index({"...", 1});
        auto ak = -euler.index({"...", 0});
        auto si = ai.sin();
        auto sj = aj.sin();
        auto sk = ak.sin();
        auto ci = ai.cos();
        auto cj = aj.cos();
        auto ck = ak.cos();
        auto cc = ci * ck;
        auto cs = ci * sk;
        auto sc = si * ck;
        auto ss = si * sk;

        auto idx = euler.sizes().slice(0, -1).vec();
        idx.push_back(3);
        idx.push_back(3);
        auto mat = torch::empty(idx, torch::kF64);
        mat.index_put_({"...", 2, 2}, cj * ck);
        mat.index_put_({"...", 2, 1}, sj * sc - cs);
        mat.index_put_({"...", 2, 0}, sj * cc + ss);
        mat.index_put_({"...", 1, 2}, cj * sk);
        mat.index_put_({"...", 1, 1}, sj * ss + cc);
        mat.index_put_({"...", 1, 0}, sj * cs - sc);
        mat.index_put_({"...", 0, 2}, -sj);
        mat.index_put_({"...", 0, 1}, cj * si);
        mat.index_put_({"...", 0, 0}, cj * ci);

        return mat;

    }

    torch::Tensor mat2euler(torch::Tensor const& rMat, string const& axes)
    {

        auto[firstAxis, parity, repetition, frame] = globs::_AXES2TUPLE.at(axes);
        auto i = firstAxis;
        auto j = globs::_NEXT_AXIS[i + parity];
        auto k = globs::_NEXT_AXIS[i - parity + 1];

        auto M = rMat.index({torch::indexing::Slice(0, 3),
                             torch::indexing::Slice(0, 3)});
        float ax, ay, az;
        if(repetition)
        {
            auto sy = (M.index({i, j}).square() + M.index({i, k}).square()).sqrt();
            if(sy.item<float>() > globs::EPS)
            {
                ax = torch::atan2(M.index({i, j}), M.index({i, k})).item<float>();
                ay = torch::atan2(sy, M.index({i, i})).item<float>();
                az = torch::atan2(M.index({j, i}), -M.index({k, i})).item<float>();
            }else
            {
                ax = torch::atan2(-M.index({j, k}), M.index({j, j})).item<float>();
                ay = torch::atan2(sy, M.index({i, i})).item<float>();
                az = 0.f;
            }
        }else
        {
            auto cy = (M.index({i, i}).square() + M.index({j, i}).square()).sqrt();
            if(cy.item<float>() > globs::EPS)
            {
                ax = torch::atan2(M.index({k, j}), M.index({k, k})).item<float>();
                ay = torch::atan2(-M.index({k, i}), cy).item<float>();
                az = torch::atan2(M.index({j, i}), M.index({i, i})).item<float>();
            }else
            {
                ax = torch::atan2(-M.index({j, k}), M.index({j, j})).item<float>();
                ay = torch::atan2( M.index({k, i}), cy).item<float>();
                az = 0.f;
            }
        }

        if (parity)
        {
            ax = -ax;
            ay = -ay;
            az = -az;
        }
        if (frame)
        {
            ax = az;
            az = ax;
        }
        return torch::tensor({ax, ay, az});
    }

    torch::Tensor quat2mat(torch::Tensor const& quat)
    {
        auto inds = torch::tensor({3, 0, 1, 2});
        auto q = quat[inds];

        auto n = torch::dot(q, q);
        if (n.item<float>() < globs::EPS)
            return torch::eye(3);

        q *=  (2 / n).sqrt();
        auto q2 = torch::outer(q, q);

        auto r1 = torch::cat({1.0 - q2[2][2] - q2[3][3], q2[1][2] - q2[3][0], q2[1][3] + q2[2][0]});
        auto r2 = torch::cat({1.0 - q2[2][2] - q2[3][3], q2[1][2] - q2[3][0], q2[1][3] + q2[2][0]});
        auto r3 = torch::cat({1.0 - q2[2][2] - q2[3][3], q2[1][2] - q2[3][0], q2[1][3] + q2[2][0]});

        return torch::stack({r1, r2, r3}).view({3, 3});
    }
};
