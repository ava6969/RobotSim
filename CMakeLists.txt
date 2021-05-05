cmake_minimum_required(VERSION 3.19)
set(CMAKE_TOOLCHAIN_FILE ~/vcpkg/scripts/buildsystems/vcpkg.cmake CACHE STRING "Vcpkg toolchain file")
project(Robosuite)

add_compile_options( -larmadillo -lmlpack -fopenmp -lprotobuf -lpthread  -fconcepts )

set(CMAKE_CXX_STANDARD 20)
set(USE_GL 1) #USE_GL==0 does not work for 131 version since there is no 'nogl' file
set(BIN_NAME ${PROJECT_NAME})

link_directories(${CMAKE_SOURCE_DIR}/mujoco/bin/)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
include_directories(environments)
include_directories(mujoco/include)

find_package(nlohmann_json CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)
find_package(OpenCV REQUIRED )

#Finding main mujoco library
if(${USE_GL})
    file(GLOB LIB_MUJOCO mujoco/bin/libmujoco[0-9][0-9][0-9].so)
else()
    file(GLOB LIB_MUJOCO mujoco/bin/libmujoco[0-9][0-9][0-9]nogl.so)
endif()

message(STATUS "GLFW lib found at: " ${GLFW})
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

add_executable(Robosuite main.cpp
        environments/robosuite/models/base.cpp environments/robosuite/models/base.h
        environments/robosuite/models/world.h
        environments/robosuite/utils/mjcf_utils.h environments/robosuite/utils/macros.h
        environments/robosuite/controllers/interpolator.cpp environments/robosuite/controllers/interpolator.h environments/robosuite/controllers/base_controller.cpp
        environments/robosuite/controllers/base_controller.h environments/robosuite/controllers/controller_factory.cpp environments/robosuite/controllers/controller_factory.h
        environments/robosuite/controllers/joint_pos.cpp environments/robosuite/controllers/joint_pos.h environments/robosuite/controllers/joint_vel.cpp environments/robosuite/controllers/joint_vel.h environments/robosuite/controllers/joint_tor.cpp
        environments/robosuite/controllers/joint_tor.h environments/robosuite/mj_sim.h

        environments/robosuite/environments/base.cpp environments/robosuite/environments/base.h environments/robosuite/environments/robot_env.cpp environments/robosuite/environments/robot_env.h
        environments/robosuite/environments/manipulation/manipulation_env.cpp environments/robosuite/environments/manipulation/manipulation_env.h environments/robosuite/environments/manipulation/single_arm_env.cpp
        environments/robosuite/environments/manipulation/single_arm_env.h
        environments/robosuite/environments/manipulation/stack.cpp environments/robosuite/environments/manipulation/stack.h

        environments/robosuite/models/objects/primitive/box.h environments/robosuite/models/objects/generated_objects.cpp environments/robosuite/models/objects/generated_objects.h
        environments/robosuite/models/objects/objects.cpp environments/robosuite/models/objects/objects.h environments/robosuite/models/objects/xml_objects.h
        environments/robosuite/models/robots/robot_model.cpp environments/robosuite/models/robots/robot_model.h
        environments/robosuite/models/robots/sawyer_robot.h
        environments/robosuite/models/robots/manipulator_model.cpp environments/robosuite/models/robots/manipulator_model.h
        environments/robosuite/models/tasks/task.cpp environments/robosuite/models/tasks/task.h
        environments/robosuite/robots/robot.cpp
        environments/robosuite/robots/robot.h environments/robosuite/robots/manipulator.cpp
        environments/robosuite/robots/manipulator.h environments/robosuite/robots/single_arm.cpp environments/robosuite/robots/single_arm.h
        environments/robosuite/scripts.h environments/robosuite/wrappers/gym_wrapper.cpp environments/robosuite/wrappers/gym_wrapper.h environments/robosuite/wrappers/wrapper.cpp
        environments/robosuite/wrappers/wrapper.h environments/robosuite/utils/globals.h
        environments/robosuite/utils/transforms.h environments/robosuite/utils/transforms.cpp
        environments/robosuite/devices/device.cpp environments/robosuite/devices/device.h
        environments/robosuite/utils/mjcf_utils.cpp environments/robosuite/utils/control_utils.cpp environments/robosuite/utils/control_utils.h

        environments/robosuite/models/mounts/mount_model.cpp environments/robosuite/models/mounts/mount_model.h environments/robosuite/models/mounts/null_mount.h environments/robosuite/models/mounts/rethink_mount.h
        environments/robosuite/models/mounts/rethink_minimal_mount.h environments/robosuite/models/grippers/gripper_factory.cpp environments/robosuite/models/grippers/gripper_factory.h
        environments/robosuite/models/grippers/gripper_model.cpp environments/robosuite/models/grippers/gripper_model.h environments/robosuite/models/grippers/gripper_tester.cpp environments/robosuite/models/grippers/gripper_tester.h
        environments/robosuite/models/grippers/rethink_gripper.h environments/robosuite/models/grippers/robotiq_three_finger_gripper.cpp environments/robosuite/models/grippers/robotiq_three_finger_gripper.h environments/robosuite/models/arenas/arena.cpp
        environments/robosuite/models/arenas/arena.h environments/robosuite/models/arenas/table_arena.cpp environments/robosuite/models/arenas/table_arena.h environments/robosuite/models/arenas/empty_arena.h environments/robosuite/models/robots/robot_factory.h
        environments/robosuite/utils/placement_samplers.cpp environments/robosuite/utils/placement_samplers.h environments/robosuite/utils/observables.cpp environments/robosuite/utils/observables.h
        environments/robosuite/utils/buffers.h environments/robosuite/utils/sensors.h environments/robosuite/utils/sensors.cpp
        )

target_link_libraries(Robosuite ${TORCH_LIBRARIES} ${LIB_MUJOCO}   nlohmann_json nlohmann_json::nlohmann_json
        pugixml   ${OpenCV_LIBS}   libGL.so libglew.so)