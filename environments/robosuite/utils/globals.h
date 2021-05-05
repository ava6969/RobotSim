//
// Created by dewe on 4/25/21.
//

#ifndef FASTDRL_GLOBALS_H
#define FASTDRL_GLOBALS_H

#include "../utils/space.h"
#include "torch/torch.h"
#include "string"
#include "vector"
#include <filesystem>

using std::string;
using std::vector;

namespace globs
{
    [[maybe_unused]] static const std::filesystem::path assets_root = std::filesystem::current_path() / "assets";
// map axes strings to/from tuples of inner axis, parity, repetition, frame
    static const  std::map<string, std::tuple<int, int, int, int>> _AXES2TUPLE = {
            {"sxyz", {0, 0, 0, 0}},
            {"sxyx", {0, 0, 1, 0}},
//            {"sxzy": (0, 1, 0, 0),
//            {"sxzx": (0, 1, 1, 0),
//            {"syzx": (1, 0, 0, 0),
//            {"syzy": (1, 0, 1, 0),
//            {"syxz": (1, 1, 0, 0),
//            {"syxy": (1, 1, 1, 0),
//            {"szxy": (2, 0, 0, 0),
//            {"szxz": (2, 0, 1, 0),
//            {"szyx": (2, 1, 0, 0),
//            {"szyz": (2, 1, 1, 0),
//            {"rzyx": (0, 0, 0, 1),
//            {"rxyx": (0, 0, 1, 1),
//            {"ryzx": (0, 1, 0, 1),
//            {"rxzx": (0, 1, 1, 1),
//            {"rxzy": (1, 0, 0, 1),
//            {"ryzy": (1, 0, 1, 1),
//            {"rzxy": (1, 1, 0, 1),
//            {"ryxy": (1, 1, 1, 1),
//            {"ryxz": (2, 0, 0, 1),
//            {"rzxz": (2, 0, 1, 1),
//            {"rxyz": (2, 1, 0, 1),
//            {"rzyz": (2, 1, 1, 1),
    };

//    auto _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

    static const auto PI = 3.1415926535;
    static const auto EPS = 4 * std::numeric_limits<float>::epsilon();
    static const auto _NEXT_AXIS = torch::tensor({1, 2, 0, 1});
    using VectF4 = std::array<float, 4>;
    using Vect3 = std::array<float, 3>;
    using RangeTorch =  Space::Range<torch::Tensor>;
    using RangeVectXf = std::vector<std::array<int, 2>>;

    static const VectF4 RED{1, 0, 0, 1};
    static const VectF4 GREEN{0, 1, 0, 1};
    static const VectF4 BLUE{0, 0, 1, 1};
    static const VectF4 CYAN{0, 1, 1, 1};
    static const VectF4 ROBOT_COLLISION_COLOR{0, 0.5, 0, 1};
    static const VectF4 MOUNT_COLLISION_COLOR {0.5, 0.5, 0, 1};
    static const VectF4 GRIPPER_COLLISION_COLOR{0, 0, 0.5, 1};
    static const VectF4 OBJECT_COLLISION_COLOR{0.5, 0, 0, 1};
    static const VectF4 ENVIRONMENT_COLLISION_COLOR{0.5, 0.5, 0, 1};

    static const vector<string> IMPEDANCE_MODES = {"fixed", "variable", "variable_kp"};
    [[maybe_unused]]
    static const vector<string> SENSOR_TYPES = {
            "touch",
            "accelerometer",
            "velocimeter",
            "gyro",
            "force",
            "torque",
            "magnetometer",
            "rangefinder",
            "jointpos",
            "jointvel",
            "tendonpos",
            "tendonvel",
            "actuatorpos",
            "actuatorvel",
            "actuatorfrc",
            "ballangvel",
            "jointlimitpos",
            "jointlimitvel",
            "jointlimitfrc",
            "tendonlimitpos",
            "tendonlimitvel",
            "tendonlimitfrc",
            "framepos",
            "framequat",
            "framexaxis",
            "frameyaxis",
            "framezaxis",
            "framelinvel",
            "frameangvel",
            "framelinacc",
            "frameangacc",
            "subtreecom",
            "subtreelinvel",
            "subtreeangmom",
            "user",
    };

    [[maybe_unused]]
    static const vector<string> MUJOCO_NAMED_ATTRIBUTES = {
            "class", "childclass", "name", "objname", "material", "texture",
            "joint", "joint1", "joint2", "jointinparent", "geom", "geom1", "geom2",
            "mesh", "fixed", "actuator", "objname", "tendon", "tendon1", "tendon2",
            "slidesite", "cranksite", "body", "body1", "body2", "hfield", "target",
            "prefix", "site",
    };

    [[maybe_unused]]
    static const std::unordered_map<string, int> IMAGE_CONVENTION_MAPPING = {
            {"opengl", 1},
            {"opencv", -1},
    };

    [[maybe_unused]]
    static const std::unordered_map<string, string> TEXTURES = {
            {"WoodRed", "red-wood.png"},
            {"WoodGreen", "green-wood.png"},
            {"WoodBlue", "blue-wood.png"},
            {"WoodLight", "light-wood.png"},
            {"WoodDark", "dark-wood.png"},
            {"WoodTiles", "wood-tiles.png"},
            {"WoodPanels", "wood-varnished-panels.png"},
            {"WoodgrainGray", "gray-woodgrain.png"},
            {"PlasterCream", "cream-plaster.png"},
            {"PlasterPink", "pink-plaster.png"},
            {"PlasterYellow", "yellow-plaster.png"},
            {"PlasterGray", "gray-plaster.png"},
            {"PlasterWhite", "white-plaster.png"},
            {"BricksWhite", "white-bricks.png"},
            {"Metal", "metal.png"},
            {"SteelBrushed", "steel-brushed.png"},
            {"SteelScratched", "steel-scratched.png"},
            {"Brass", "brass-ambra.png"},
            {"Bread", "bread.png"},
            {"Can", "can.png"},
            {"Ceramic", "ceramic.png"},
            {"Cereal", "cereal.png"},
            {"Clay", "clay.png"},
            {"Dirt", "dirt.png"},
            {"Glass", "glass.png"},
            {"FeltGray", "gray-felt.png"},
            {"Lemon", "lemon.png"}};

    template<typename KeyType, typename ValueType>
    static vector<KeyType> find_keys(std::unordered_map<KeyType, ValueType> _map)
    {
        std::vector<KeyType> keys;
        std::transform(begin(_map), end(_map), back_inserter(keys), [](auto& pair_) -> KeyType
        {
            return pair_.first;
        });
        return keys;
    }

    static const vector<string>  ALL_TEXTURES = find_keys(TEXTURES);



}



#endif //FASTDRL_GLOBALS_H
