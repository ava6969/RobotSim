//
// Created by dewe on 4/27/21.
//

#include "gripper_factory.h"
#include "rethink_gripper.h"

std::unique_ptr<GripperModel> GripperFactory::create_gripper(GripperFactory::GripperType type, int id) {

    switch (type) {

        case RethinkGripper:
            return std::make_unique<class RethinkGripper>(id);

        default:
            return {};
    }

}
