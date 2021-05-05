//
// Created by dewe on 4/26/21.
//

#include "device.h"

struct ControllerState
{

};

void Keyboard::startControl() {

    _reset_internal_state();
    resetState = 0;
    enabled = true;
}

ControllerState Keyboard::get_controller_state() {
    return ControllerState();
}

void Keyboard::_reset_internal_state()
{
    rotation = torch::tensor({{-1.0, 0.f, 0.f}, {0., 1., 0.}, {0., 0., -1.}});
    rawDRotation = torch::zeros(3);
    lastDRotation = torch::zeros(3);
    pos= torch::zeros(3);
    lastPos = torch::zeros(3);
    grasp = false;

}

void Keyboard::onRelease(GLFWwindow *window, int key, int scancode, int action, int mods) {

    if(key == GLFW_KEY_SPACE)
        grasp = !grasp;
    else if(key == GLFW_KEY_Q)
    {
        resetState=1;
        enabled= false;
        _reset_internal_state();
    }
}

void Keyboard::onPress(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    switch (key) {

        case GLFW_KEY_W:
            pos[0] -= posStep * pos_sensitivity;
            break;

        case GLFW_KEY_S:
            pos[0] += posStep * pos_sensitivity;
            break;

        case GLFW_KEY_A:
            pos[1] -= posStep * pos_sensitivity;
            break;

        case GLFW_KEY_D:
            pos[1] += posStep * pos_sensitivity;
            break;

        case GLFW_KEY_F:
            pos[2] -= posStep * pos_sensitivity;
            break;

        case GLFW_KEY_Z:
            pos[2] += posStep * pos_sensitivity;
            break;


    }
}
