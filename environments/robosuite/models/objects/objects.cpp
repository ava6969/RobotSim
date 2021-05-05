//
// Created by dewe on 4/24/21.
//

#include "objects.h"

#include <utility>
#include "../../utils/mjcf_utils.h"


const unordered_map<string, vector<int>> MujocoObject::GEOMTYPE2GROUP = { {"collision", {0}},
                                                                          {"visual", {1}},
                                                                          {"all", {0, 1}} };

vector<string> MujocoObject::GEOM_GROUPS = {};

MujocoObject::MujocoObject(string objType, bool duplicateCollisionGeoms):obj_type(std::move(objType)), duplicate_collision_geoms(duplicateCollisionGeoms)
{

    asset.set_name("asset");

    if(GEOM_GROUPS.empty())
    {
        for(const auto& entry: GEOMTYPE2GROUP)
            GEOM_GROUPS.push_back(entry.first);
    }
    bool available = mjcfUtils::found(GEOM_GROUPS ,objType);
    assert(available && "object type must be in geoms group");

}

void MujocoObject::mergeAssets(MujocoXML *other)
{
    for(auto& _asset :  other->Asset().children())
    {
        if(!mjcfUtils::findElements(this->asset, _asset.name(),
                                    {{"name", _asset.attribute("name").value()}}, true))
        {
            this->asset.append_copy(_asset);
        }
    }
}

pugi::xml_node MujocoObject::getObj() {
    assert(!_obj && "Object XML tree has not been generated yet!");
    return _obj;
}

void MujocoObject::getObjectProperties()
{
    auto _elements = mjcfUtils::sortElements(getObj(), {}, {}, {});
    assert(_elements["root_body"].size() == 1);

//    _elements["root_body"] = _elements["root_body"].front();
    auto b = _elements["root_body"];
//    b.insert(end(b), begin(bodies()), end(bodies()));
//
//    _elements["bodies"] =


#ifdef USING_INSTANCE_RANDOMIZATION

    auto res = mjcfUtils::add_material(getObj(), namingPrefix());
        if(res.used)
        {
            asset.append_copy(res.tex_element);
            asset.append_copy(res.mat_element);
        }
#endif

    mjcfUtils::add_prefix(getObj(), this->namingPrefix(), [&](auto tag){
        return this->excludeFromPrefixing(tag);
    }, {}, {});


}

MujocoXMLObject::MujocoXMLObject(string fileName,
                                 string name,
                                 std::vector<std::unordered_map<string, string>> const& joints,
                                 string objType,
                                 bool duplicateCollisionGeoms): MujocoObject(std::move(objType), duplicateCollisionGeoms),
                                                                MujocoXML(std::move(fileName))
                                                                {

    this->_name = std::move(name);

    if(!joints.empty() && mjcfUtils::found<string>(joints.front(), "default"))
    {
        jointSpecs = {getJointAttribTemplate()};
    }else
    {
        jointSpecs = joints;
    }

    int i = 0;
    for(auto& joint_spec : jointSpecs)
    {
        if(!mjcfUtils::found<string>(joint_spec, "name"))
        {
            joint_spec["name"] = "joint" + std::to_string(i);
        }
        i++;
    }

    _obj = getObjectSubTree();

    getObjectProperties();

}

pugi::xml_node MujocoXMLObject::getObjectSubTree() {
    return pugi::xml_node();
}

bool MujocoXMLObject::excludeFromPrefixing(const string &inp) {
    return false;
}

void MujocoXMLObject::getObjectProperties() {
    MujocoObject::getObjectProperties();
}

vector<std::pair<pugi::xml_node, pugi::xml_node>>
MujocoXMLObject::getGeoms(pugi::xml_node root, pugi::xml_node _parent) {
    return vector<std::pair<pugi::xml_node, pugi::xml_node>>();
}

torch::Tensor MujocoXMLObject::bottomOffset() {
    return torch::Tensor();
}

torch::Tensor MujocoXMLObject::topOffset() {
    return torch::Tensor();
}

double MujocoXMLObject::horizontalRadius() {
    return 0;
}

pugi::xml_node MujocoXMLObject::duplicateVisualFromCollision(pugi::xml_node element) {
    return pugi::xml_node();
}

MujocoGeneratedObject::MujocoGeneratedObject(string objType, bool duplicateCollisionGeoms) : MujocoObject(objType,
                                                                                                          duplicateCollisionGeoms) {

}
