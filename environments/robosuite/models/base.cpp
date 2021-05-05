

#include "base.h"
#include "../mj_sim.h"
#include "../utils/macros.h"
#include "../utils/mjcf_utils.h"

using Element = pugi::xml_node;
using NestedDict =  std::unordered_map<string, unordered_map<string, string>>;

using namespace pugi;

MujocoXML::MujocoXML(string _fileName): fileName(std::move(_fileName)) {

    //folder = boost::filesystem::path
    root.load_file(fileName.c_str());
    folder = std::filesystem::current_path();

    worldBody = createDefaultElement("worldbody");
    actuator = createDefaultElement("actuator");
    sensor = createDefaultElement("sensor");
    asset = createDefaultElement("asset");
    tendon = createDefaultElement("tendon");
    equality = createDefaultElement("equality");
    contact = createDefaultElement("contact");

    auto default_ = createDefaultElement("default");
    auto default_classes = getDefaultClasses(default_);
    replaceDefaultInline(default_classes);

    resolveAssetDependency();
}

pugi::xml_node  MujocoXML::createDefaultElement(const string &name)
{
    auto found_node  = root.child(name.c_str());
    if (found_node)
    {
        return found_node;
    }
    root.append_child(name.c_str());

    assert(!root->child(name.c_str()).empty());

    return root.child(name.c_str());
}

void MujocoXML::resolveAssetDependency()
{

    mjcfUtils::GetAllPredicate walker([](pugi::xml_node& node) {return node.attribute("file"); }, false);
    asset.traverse(walker);
    for (auto node: walker.nodes)
    {
        auto path = folder / node.attribute("file").value();
        node.attribute("file").set_value(path.c_str());
    }

}

NestedDict MujocoXML::getDefaultClasses(pugi::xml_node _default)
{
    // dont fully understand now
    NestedDict defaultDic;
    for(auto cls : _default)
    {
        auto key = cls.attribute("class").value();
        defaultDic[key] = unordered_map<string, string>{};
        for (auto child : cls)
            defaultDic[key][child.name()] = child.value();
    }

    return defaultDic;
}

void MujocoXML::merge(const vector<MujocoXML *>& others, string const&mergeBody)
{
    for(auto other : others)
    {
        if(not mergeBody.empty())
        {
            auto root_ = mergeBody == "default" ? this->worldBody : mjcfUtils::findElements(this->worldBody, "body", {{"name", mergeBody}}, true);
            for (auto const& body : other->worldBody)
            {
                root_.append_copy(body);
            }
        }
        mergeAssets(other);
        for(auto oneActuator: other->actuator)
        {
            actuator.append_copy(oneActuator);
        }
        for(auto oneSensor: other->sensor)
        {
            sensor.append_copy(oneSensor);
        }
        for(auto oneTendon: other->tendon)
        {
            tendon.append_copy(oneTendon);
        }
        for(auto oneEquality: other->equality)
        {
            equality.append_copy(oneEquality);
        }
        for(auto const& oneContact: other->contact)
        {
            contact.append_copy(oneContact);
        }
    }
}

void MujocoXML::merge(MujocoXML * others, string const &mergeBody) {
    merge(vector<MujocoXML*>{others}, mergeBody);
}

_mjModel* MujocoXML::getModel()
{
    // save model  to xml file as root_name.xml
    auto tmp_dir = std::filesystem::temp_directory_path();
    std::filesystem::path save_dir;
    save_dir = tmp_dir / "models";
    std::filesystem::create_directory(save_dir);
    auto fPath = save_dir / (string(root.name()) + ".xml");
    root.save_file(fPath.c_str());

    // load it
    return mj_loadXML(fPath.c_str()); // todo: remember to reinstall mujoco

}

string MujocoXML::getXML() {

    std::stringstream ss;
    root.save(ss);
    return ss.str();
}

void MujocoXML::saveModel(const string & _fileName)
{
    root.save_file(_fileName.c_str());
}

void MujocoXML::mergeAssets(MujocoXML *other)
{
    for (pugi::xml_node _asset: other->asset)
    {
        if ( mjcfUtils::findElements(_asset,
                                     _asset.name(),
                                     {{"name",string(asset.attribute("name").value())}},
                                     true).empty())
        {
            asset.append_copy(_asset); // todo: maybe move -> figure out
        }
    }
}

vector<string> MujocoXML::getElementNames(xml_node _root, const string &elementType)
{
    std::vector<string> namesAttributes;
    for(pugi::xml_node child : _root)
    {
        if (strcmp(child.name(), elementType.c_str()) == 0)
        {
            namesAttributes.emplace_back(child.attribute("name").value());
        }
        auto attrs = std::move(getElementNames(child, elementType));
        namesAttributes.insert(end(namesAttributes), begin(attrs), end(attrs));
    }

    return namesAttributes;
}

void MujocoXML::replaceDefaultInline(const NestedDict &defaultDic, pugi::xml_node _root)
{
    if(_root.empty())
    {
        _root = root.root();
    }
    auto cls_name = _root.attribute("class").name();
    if (cls_name)
    {
        auto tagAttrs = defaultDic.find(cls_name);
        if(tagAttrs != end(defaultDic))
        {
            for(auto& entry : tagAttrs->second)
            {
                if(_root.attribute(entry.first.c_str()).empty())
                {
                    _root.attribute(entry.first.c_str()).set_value(entry.second.c_str());
                }
            }
        }
    }

    for(auto child: _root)
    {
        replaceDefaultInline(defaultDic, _root);
    }
}

string MujocoModel::correctNaming(string name)
{
    return excludeFromPrefixing(name) ? name : namingPrefix() + name;
}

vector<string> MujocoModel::correctNaming(vector<string> names)
{
    vector<string> res;
    std::transform(begin(names), end(names), back_inserter(res), [&](string const& name)
    {
        return correctNaming(name);
    });
    return res;
}

unordered_map<string, vector<string>> MujocoModel::correctNaming(unordered_map<string, vector<string>> names)
{
    std::for_each(begin(names), end(names), [&](auto const& pair)
    {

        names[pair.first] = pair.second;
    });

    return names;
}

void MujocoModel::setSitesVisibility( MjSim *sim, bool visible) {

    for(auto& site: this->sites())
    {
        auto param = MjModelParameter{mjOBJ_SITE, site, 3};
        auto alpha = sim->get<float>(param);
        if( (visible && alpha < 0) || (!visible && alpha > 0))
            sim->put(param, -alpha);

    }
}


MujocoXMLModel::MujocoXMLModel(const string &fname, int idn): MujocoXML(fname),idn(idn)
{
    elements = mjcfUtils::sortElements(root.root(), {}, {}, {});

    assert(elements.find("root_body") != end(elements));
    assert(elements["root_body"].size() == 1 && "Invalid number of root bodies found for robot model. Expected 1, got " + elements["root_body"].size());

    //elements["bodies"]

}

void MujocoXMLModel::initialize()
{

    if(initialized)
        return;

    mjcfUtils::add_prefix(root.root(), this->namingPrefix(), [&](auto tag){
        return this->excludeFromPrefixing(tag);
    }, {}, {});

    mjcfUtils::recolorCollisionGeoms(this->worldBody, this->contactGeomRGBA());


#ifdef USING_INSTANCE_RANDOMIZATION

    auto res = mjcfUtils::add_material(worldBody, namingPrefix());
        if(res.used)
        {
            asset.append_copy(res.tex_element);
            asset.append_copy(res.mat_element);
        }
#endif

    initialized = true;
}

