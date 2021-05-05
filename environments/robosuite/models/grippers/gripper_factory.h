//
// Created by dewe on 4/27/21.
//

#ifndef FASTDRL_GRIPPER_FACTORY_H
#define FASTDRL_GRIPPER_FACTORY_H


#include <memory>
#include "gripper_model.h"


class GripperFactory {


enum GripperType
{
    RethinkGripper,
    PandaGripper,
    JacoThreeFingerGripper,
    JacoThreeFingerDexterousGripper,
    WipingGripper,
    Robotiq85Gripper,
    Robotiq140Gripper,
    RobotiqThreeFingerGripper,
    RobotiqThreeFingerDexterousGripper,
    None
};


public:

    std::unique_ptr<GripperModel> create_gripper(GripperType type, int id) ;



};


#endif //FASTDRL_GRIPPER_FACTORY_H
