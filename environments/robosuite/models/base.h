//
// Created by dewe on 4/23/21.
//

#ifndef FASTDRL_WORLD_H
#define FASTDRL_WORLD_H

#include <filesystem>
#include "string"
#include "vector"
#include "mujoco.h"
#include "unordered_map"
#include "pugixml.hpp"
#include "memory"
#include "torch/torch.h"

using std::unordered_map;
using std::vector;
using std::string;

/*
    Base class of Mujoco xml file
    Wraps around ElementTree and provides additional functionality for merging different models.
    Specially, we keep track of <worldbody/>, <actuator/> and <asset/>

    When initialized, loads a mujoco xml from file.

    Args:
    fname (str): path to the MJCF xml file.
*/
class MujocoXML {



    using StringVectMap = unordered_map<string, vector<string>>;
    using Element = pugi::xml_node;
    using ElementDict = std::unordered_map<std::string, vector<Element>>;
    using StrDict = std::unordered_map<std::string, std::string>;
    using Attributes = vector<pugi::xml_attribute>;

    using NestedDict =  std::unordered_map<string, unordered_map<string, string>>;


protected:
    string fileName;
    std::filesystem::path folder;
    pugi::xml_node worldBody, actuator, sensor, asset, tendon, equality, contact;

protected:
    pugi::xml_document root;
public:
    explicit MujocoXML(string fileName);

    void resolveAssetDependency();

    pugi::xml_node createDefaultElement(std::string const& name);

    void merge(const vector<MujocoXML*>& others, string const &mergeBody);
    void merge(MujocoXML* others, string const &mergeBody);

    _mjModel* getModel();

    pugi::xml_node Asset() const { return asset; }

    string getXML();

    void saveModel(string const& fileName);

    void mergeAssets(MujocoXML* other);

    vector<string> getElementNames(pugi::xml_node _root, std::string const& elementType);

    static NestedDict getDefaultClasses(pugi::xml_node root);

    void replaceDefaultInline(NestedDict const& defaultDic, pugi::xml_node root={});

    inline virtual std::string name() { return root.attribute("model").name(); }

};


class MujocoModel
{

public:
    MujocoModel()=default;

    string correctNaming( string name);
    vector<string> correctNaming( vector<string> names);

    unordered_map<string, vector<string>>  correctNaming( unordered_map<string,vector<string>> names);

    void setSitesVisibility( struct MjSim* sim, bool visible);

    virtual bool excludeFromPrefixing(string const&  inp) = 0;
    virtual string name() = 0;
    virtual string namingPrefix() = 0;
    virtual string rootBody() = 0;
    virtual vector<string> bodies() = 0;
    virtual vector<string> actuators() = 0;
    virtual vector<string> sites() = 0;
    virtual vector<string> sensors() = 0;
    virtual vector<string> contactGeoms() = 0;
    virtual vector<string> visualGeoms() = 0;
    virtual unordered_map<string, vector<string>>   importantGeoms() = 0;
    virtual unordered_map<string, vector<string>>   importantSites() = 0;
    virtual unordered_map<string, vector<string>>   importantSensors() = 0;
    virtual torch::Tensor bottomOffset() = 0;
    virtual  torch::Tensor  topOffset() = 0;
    virtual double horizontalRadius() = 0;

};


class MujocoXMLModel : public MujocoXML, public MujocoModel
{

    using Element = pugi::xml_node;

protected:
    int idn;
    vector<string> _bodies, _joints, _actuators, _sites, _sensors, _contact_geoms, _visual_geoms, _root_body, mount;
    torch::Tensor _base_offset{};
    bool initialized{false};
    torch::Tensor rotationOffset;

    std::unordered_map<std::string, vector<Element>> elements;
public:

    explicit MujocoXMLModel(const string& fname, int idn=0);

    virtual void initialize();

    bool excludeFromPrefixing(string const& inp) override { return false; }

    inline   torch::Tensor baseOffset() { return _base_offset; }

    inline string name()  override { return typeid(this).name() + std::to_string(idn);  }

    [[nodiscard]] string namingPrefix() override { return std::to_string(idn) + "_"; }

    string rootBody() { return correctNaming(_root_body.front()); }

    vector<string> bodies() { return correctNaming(_bodies); }

    vector<string> joints() { return correctNaming(_joints); }

    vector<string> actuators() { return correctNaming(_actuators); }

    vector<string> sites() { return correctNaming(_sites); }

    vector<string> sensors() { return correctNaming(_sensors); }

    vector<string> contactGeoms() { return correctNaming(_contact_geoms); }

    vector<string> visualGeoms() { return correctNaming(_visual_geoms); }

    unordered_map<string, vector<string>>  importantSites() { return correctNaming(_importantSites()); }

    unordered_map<string, vector<string>>  importantGeoms() { return correctNaming(_importantGeoms()); }

    unordered_map<string, vector<string>> importantSensors() { return correctNaming(_importantSensors()); }

    virtual unordered_map<string, vector<string>>  _importantSites() { return {{}}; }

    virtual unordered_map<string, vector<string>>   _importantGeoms() { return {{}}; }

    virtual unordered_map<string, vector<string>> _importantSensors() { return {{}}; }

    virtual std::array<float, 4> contactGeomRGBA() = 0;

    torch::Tensor bottomOffset() override {  return _base_offset; }

    virtual torch::Tensor topOffset() = 0;

    inline std::unordered_map<std::string, vector<Element>>& getElement() { return elements; }

    torch::Tensor getRotationOffset() {  return rotationOffset; }


};

#endif //FASTDRL_WORLD_H
