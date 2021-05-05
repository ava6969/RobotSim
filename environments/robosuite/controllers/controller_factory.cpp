//
// Created by dewe on 4/24/21.
//

#include <cassert>
#include <fstream>
#include "../mj_sim.h"
#include "filesystem"
#include "controller_factory.h"
#include "joint_pos.h"

namespace CtrlFactory {

    json loadControllerConfig(std::string const& custom_fpath, std::string defaultCtrl)
    {
        auto custom_fpath_modified  =custom_fpath;
        if(!defaultCtrl.empty())
        {
            //load controller
            // assert();
            auto file = defaultCtrl  + ".json";
            custom_fpath_modified = std::filesystem::current_path() / ".." / "controllers" / "config" / file;
        }
        json controllerConfig;
        assert(!custom_fpath.empty() && "Error: Either custom_fpath or default_controller must be specified!" );
        try
        {
            std::ifstream i(custom_fpath_modified);
            i >> controllerConfig;

        }catch(std::exception const& exp)
        {
            printf("%s, Error opening controller filepath at: %s}. "
                  "Please check filepath and try again.", exp.what(), custom_fpath.c_str());
        }
        return controllerConfig;
    }


    std::unique_ptr<BaseController>  controllerFactory(MjSim* sim,
                                                       std::string const& name,
                                                       CtrlParameters const& params)
    {
        std::unique_ptr<Interpolator> interpolator;

        if(params.interpolator == "linear")
        {
            interpolator = std::make_unique<LinearInterpolator>(params.nDim,
                                                                (1 / sim->model->opt.timestep),
                                                                params.policyFreq,
                                                                params.rampRatio);
        }

        if (name == "JOINT_POSITION")
        {
            //return std::make_unique<JointPositionController>(sim);
        }

        return {};
    }
};