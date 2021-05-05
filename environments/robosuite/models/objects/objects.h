//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_OBJECTS_H
#define FASTDRL_OBJECTS_H

#include "../base.h"



class MujocoObject : public MujocoModel {

protected:
    pugi::xml_node asset, _obj;
    bool duplicate_collision_geoms;
    string obj_type, _name;
    vector<string> _bodies, _joints, _actuators, _sites, _sensors, _contact_geoms, _visual_geoms, _root_body, mount;

public:

    static const unordered_map<string, vector<int>> GEOMTYPE2GROUP;

    static vector<string> GEOM_GROUPS;

    MujocoObject(string objType, bool duplicateCollisionGeoms);

    void mergeAssets(MujocoXML* other);

    pugi::xml_node getObj();

    virtual pugi::xml_node getObjectSubTree() = 0;

    virtual void getObjectProperties();

    string name() override { return _name; }

    string namingPrefix() override { return _name + "_"; }

    string rootBody() override { return correctNaming(_root_body.front()); }

    vector<string> bodies() override { return correctNaming(_bodies); }

    virtual vector<string> joints() { return correctNaming(_joints); }

    vector<string> actuators() override { return correctNaming(_actuators); }

    vector<string> sites() override { return correctNaming(_sites);}

    vector<string> sensors() override { return correctNaming(_sensors); }

    vector<string> contactGeoms() override { return correctNaming(_contact_geoms); }

    vector<string> visualGeoms() override { return correctNaming(_visual_geoms); }

    unordered_map<string, vector<string>> importantSites() override { return {{"obj", {namingPrefix() + "default_site"}}};}

    virtual unordered_map<string, string> getSiteAttribTemplate() { return {{"pos", "0 0 0"},
                                                                    {"size", "0.002 0.002 0.002"},
                                                                    {"rgba", "1 0 0 1"},
                                                                    {"type", "sphere"},
                                                                    {"group", "0"}};
    }

    virtual unordered_map<string, string> getJointAttribTemplate() { return {{"type", "free"}};}

};


class MujocoXMLObject : public MujocoObject, MujocoXML
{

    std::vector<std::unordered_map<string, string>> jointSpecs;
public:
    MujocoXMLObject(string fileName,
                    string name,
                    std::vector<std::unordered_map<string, string>> const& joints= { {"default", ""}},
                    string objType="all",
                    bool duplicateCollisionGeoms=true);

    pugi::xml_node getObjectSubTree() override;

    bool excludeFromPrefixing(string const& inp) override;

    void getObjectProperties() override;

    static pugi::xml_node duplicateVisualFromCollision(pugi::xml_node element);

    vector<std::pair<pugi::xml_node, pugi::xml_node>> getGeoms( pugi::xml_node root, pugi::xml_node _parent={});


    torch::Tensor bottomOffset() override;

    torch::Tensor  topOffset() override;

    double horizontalRadius() override;

};

class MujocoGeneratedObject : public MujocoObject
{

public:
    MujocoGeneratedObject(string objType="all", bool duplicateCollisionGeoms= true);
};

#endif //FASTDRL_OBJECTS_H
