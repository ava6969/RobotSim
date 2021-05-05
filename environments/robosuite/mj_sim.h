//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_MJ_SIM_H
#define FASTDRL_MJ_SIM_H

#include "robosuite/utils/space.h"
#include "mujoco.h"
#include "vector"
#include "functional"
#include "torch/torch.h"
#include "unordered_map"

using std::string;
using std::vector;
using std::unordered_map;

#define m3 3*sizeof(mjtNum)
#define m9 9*sizeof(mjtNum)



struct MjModelParameter
{
    int type;
    string name;
    int offset{0};
};


using RangeVectXf = Space::Range<vector<float>>;
struct MjSim {

    mjData* data{};
    mjModel* model{};
    uint nSubSteps=1;

    vector<std::array<float, 4>> sites_rgba;


    MjSim(mjModel* m, uint nSubSteps):data(mj_makeData(m)), model(m), nSubSteps(nSubSteps){

        sites_rgba.resize(model->nsite);
        for (int i = 0; i < model->nsite; i++) {
            memcpy(sites_rgba[i].data(), &model->site_rgba[i * 4], sizeof(float) * 4);
        }
    }

    void forward()
    {
        mj_forward(model, data);
    }

    mjData* getState()
    {
        return data;
    }

    torch::Tensor render(int width, int height, std::string camName="", bool depth=false)
    {
        return {};
    }

    void reset()
    {
        data = mj_makeData(model);
    }

    int save(string const& fName)
    {
        char error[1000];
        if( !mj_saveLastXML(fName.c_str(), model, error, 1000) )
            return finish(error, model, data);
        return 0;
    }

    ~MjSim()
    {
        mj_deleteData(data);
        mj_deleteModel(model);

    }

    // deallocate and print message
    int finish(const char* msg = 0, mjModel* m = 0, mjData* d = 0)
    {
        // deallocated everything
        if( d )
            mj_deleteData(d);
        if( m )
            mj_deleteModel(m);
        mj_deactivate();

        // print message
        if( msg )
            printf("%s\n", msg);

        return 0;
    }

    int getJointQposAddr(string const& name)
    {
        int id = jointName2ID(name);
        return model->jnt_qposadr[id];
    }

    std::tuple<torch::Tensor, torch::Tensor> actCtrlRange()
    {
        auto ctrl_range = torch::from_blob(model->jnt_range, {model->nu * 2}).view({model->nu, 2});

        return {
            ctrl_range.gather(1, torch::zeros(ctrl_range.size(0))),
            ctrl_range.gather(1, torch::zeros(ctrl_range.size(1)))
        };
    }

    torch::Tensor rawActCtrlRange()
    {
        return  torch::from_blob(model->jnt_range, {model->nu * 2}).view({model->nu, 2});

    }

    void actCtrl(int idx, double applied_gripper_action)
    {
        data->ctrl[idx] = applied_gripper_action;
    }

    void actCtrl(torch::Tensor idx, torch::Tensor applied_gripper_action)
    {
        int i = 0;
        while(i < idx.size(0))
        {
            data->ctrl[idx[i].item<int>()] = applied_gripper_action[i].item<double>();
            i++;
        }

    }

    auto jntRange()
    {
        return torch::from_blob(model->jnt_range, {model->nq * 2}).view({model->nq, 2});
    }

    auto sensorDim()
    {
        return torch::from_blob(model->sensor_dim, {model->nsensor});
    }

    auto sensorData()
    {
        return torch::from_blob(data->sensordata, {model->nsensordata});
    }

    int sensorName2ID(string const& name)
    {
        return mj_name2id(model, mjOBJ_SENSOR, name.c_str());
    }

    int getJointQvelAddr(string const& name)
    {
        int id = jointName2ID(name);
        return model->jnt_dofadr[id];
    }

    int jointName2ID(string const& name)
    {
        return mj_name2id(model, mjOBJ_JOINT, name.c_str());
    }

    int actName2ID(string const& name)
    {
        return mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    }

    torch::Tensor getBodyXPos(std::string str)
    {
        int id =mj_name2id(model, mjOBJ_BODY, str.c_str());
        return torch::from_blob( &data->xpos[id], {3});
    }

    torch::Tensor getBodyXMat(std::string str)
    {
        int id =mj_name2id(model, mjOBJ_BODY, str.c_str());
        return torch::from_blob( &data->xmat[id], {9});
    }

    inline torch::Tensor qpos() const
    {
        return  torch::from_blob(data->qpos, {model->nq});
    }

    inline void setQPos(torch::Tensor _indices, torch::Tensor _values) const
    {
        for(int i = 0; i < _indices.size(0); i++)
            data->qpos[_indices[i].item<int>()] = _values[i].item<double>();
    }


    inline torch::Tensor qvel() const
    {
        return  torch::from_blob(data->qvel, {model->nv});
    }

    inline torch::Tensor qfrc_bias() const
    {
        return  torch::from_blob(data->qfrc_bias, {model->nv});
    }

    inline std::tuple<torch::Tensor, torch::Tensor>  get_site_jac(int site_id) const
    {
        auto& DOF = model->nv;
        torch::Tensor jacp = torch::empty({3 * DOF}, torch::kF64);
        torch::Tensor jacr = torch::empty({3 * DOF}, torch::kF64);

        mj_jacSite(model, data, jacp.data_ptr<double>(), jacr.data_ptr<double>(), site_id);
        return {jacp, jacr};
    }

    inline std::tuple<torch::Tensor, torch::Tensor> site_xvel(int site_id) const
    {
        auto& DOF = model->nv;
        auto[jacp, jacr] = get_site_jac(site_id);
        return {
            jacp.view({3, DOF}).dot(qvel()),
            jacr.view({3, DOF}).dot(qvel())
        };
    }

    inline torch::Tensor massMatrix() const
    {
        auto& dof = model->nv;
        torch::Tensor massMatrix = torch::empty({dof * dof}, torch::kF64);
        mj_fullM(model, massMatrix.data_ptr<double>(), data->qM);
        // todo check if garbage
        return massMatrix.view({dof, dof });
    }


    template<typename T> // todo - perform specialization
    T get(MjModelParameter const& param)
    {
        auto idx = mj_name2id(model, param.type, param.name.c_str());
        switch (param.type)
        {
            case mjOBJ_SITE:
                return model->site_rgba[idx * sizeof(T)*4 + param.offset];
                break;

            default:
                return 0;
        }

    }

    template<typename T> // todo - perform specialization
    void put(MjModelParameter const& param, T value)
    {
        auto idx = mj_name2id(model, param.type, param.name.c_str());
        switch (param.type)
        {
            case mjOBJ_SITE:
                model->site_rgba[idx * sizeof(T)*4 + param.offset] = value;
                break;

        }
    }

    inline int siteName2ID(std::string const& name) const
    {
        return mj_name2id(model, mjOBJ_SITE, name.c_str());
    }

};


#endif //FASTDRL_MJ_SIM_H
