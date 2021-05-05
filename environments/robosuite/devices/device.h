//
// Created by dewe on 4/26/21.
//

#ifndef FASTDRL_DEVICE_H
#define FASTDRL_DEVICE_H

#include <glfw3.h>
#include "string"
#include "torch/torch.h"

struct ControllerState;
class Device {

public:
    Device()= default;
    virtual void startControl() = 0;
    virtual ControllerState get_controller_state() = 0;
};


class Keyboard : public Device{

    float pos_sensitivity,rot_sensitivity;
    float resetState = 0;
    bool enabled = false;
    float posStep = 0.05;
    bool grasp{false};
    torch::Tensor lastPos, pos, lastDRotation, rawDRotation, rotation;

public:
    explicit Keyboard( float pos_sensitivity, float rot_sensitivity):pos_sensitivity(pos_sensitivity), rot_sensitivity(rot_sensitivity){

        _display_controls();
        _reset_internal_state();
    }
    void startControl() override;

    static void print_command(std::string _char, std::string const& info)
    {
        _char += std::string(" ", 10 - _char.size());
        printf("%s\t%s", _char.c_str(), info.c_str());
    };

    static void _display_controls()
    {
        printf("\n");
        print_command("Keys", "Command");
        print_command("q", "reset simulation");
        print_command("spacebar", "toggle gripper (open/close)");
        print_command("w-a-s-d", "move arm horizontally in x-y plane");
        print_command("r-f", "move arm vertically");
        print_command("z-x", "rotate arm about x-axis");
        print_command("t-g", "rotate arm about y-axis");
        print_command("c-v", "rotate arm about z-axis");
        print_command("ESC", "quit");
        printf("\n");
    }

    void _reset_internal_state();
    ControllerState get_controller_state() override;
    void onRelease(GLFWwindow* window, int key, int scancode, int action, int mods);
    void onPress(GLFWwindow* window, int key, int scancode, int action, int mods);
};


#endif //FASTDRL_DEVICE_H
