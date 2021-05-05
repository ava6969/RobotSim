//
// Created by dewe on 4/26/21.
//

//
// Created by dewe on 4/24/21.
//

#ifndef FASTDRL_MJCF_UTILS_H
#define FASTDRL_MJCF_UTILS_H

#include "mjcf_utils.h"
#include "../mj_sim.h"

namespace mjcfUtils
{

    string arrayToString_(at::Tensor const& tensor) {

        assert(tensor.sizes().size() == 1);
        std::stringstream ss;
        int i = 0;
        while ( i < tensor.size(0))
            ss << tensor[i++];
        return ss.str();
    }

    std::string xmlPathCompletion(std::filesystem::path const& xmlPath)
    {
        return xmlPath.string().front() == '/' ? xmlPath : globs::assets_root / xmlPath;
    }

    struct CustomMaterial
    {
        unordered_map<string, string> tex_attribute, mat_attribute;
        string name{};
        bool shared{};

        CustomMaterial()=default;
        CustomMaterial(globs::VectF4 const& texture,
                       string const& textName,
                       string matName,
                       unordered_map<string, string> const& _tex_attribute={},
                       unordered_map<string, string> const& _mat_attribute={},
                       bool _shared=false): name(std::move(matName)), shared(_shared),
                                            tex_attribute(_tex_attribute), mat_attribute(_mat_attribute)
        {
            mat_attribute["name"] = name;
            mat_attribute["texture"] = textName;
            tex_attribute["name"] = textName;

            if(!texture.empty())
            {

                auto tex = cv::Mat(100, 100, CV_8UC4);
                tex.setTo(texture);
                auto tmp_dir = std::filesystem::temp_directory_path();
                std::filesystem::path save_dir;
                save_dir = tmp_dir / "robosuite_temp_tex";
                std::filesystem::create_directory(save_dir);
                auto fPath = save_dir / (textName + ".png");
                tex_attribute["file"] = fPath;
            }
        }

        CustomMaterial(string const& texture,
                       string textName,
                       string matName,
                       unordered_map<string, string> const& _tex_attribute={},
                       unordered_map<string, string> const& _mat_attribute={},
                       bool _shared= false):name(std::move(matName)), shared(_shared),
                                            tex_attribute(_tex_attribute), mat_attribute(_mat_attribute)
        {
            assert(std::find(begin(ALL_TEXTURES), end(ALL_TEXTURES), texture) != end(ALL_TEXTURES));
            mat_attribute["name"] = name;
            mat_attribute["texture"] = textName;
            tex_attribute["name"] = std::move(textName);

            tex_attribute["file"] = xmlPathCompletion(std::filesystem::path("textures") / globs::TEXTURES.at(texture));



        }
    };

    void setAlpha(pugi::xml_node node, float alpha)
    {
//        node->find_node([](){
//
//        });
    }

    pugi::xml_node newElement(string const& tag,
                              string const& name,
                              Attributes const& attributes)
    {
        // constructs a new node from attributes
        pugi::xml_node ele;

//        ele.append_attribute(attributes[0]);

        return  ele;
    }

    pugi::xml_node new_joint(string const& name, Attributes const& attributes)
    {
        return newElement("joint", name, attributes);
    }

    pugi::xml_node new_actuator(string const& name, string const& joint,  Attributes const& attributes, string const& act_type="actuator")
    {
        pugi::xml_node  element = newElement(act_type, name, attributes);
//        element.prepend_attribute("joint", joint); set attribute
        return element;
    }

    pugi::xml_node new_site(std::string const& name,
                            Attributes const& attributes,
                            vector<float> size,
                            globs::Vect3 pos={0, 0, 0},
                            int group=0)
    {
        return newElement("geom", name, attributes);
    }

    pugi::xml_node new_body(std::string const& name,
                            Attributes const& attributes,
                            globs::Vect3 pos={0, 0, 0})
    {
        return newElement("body", name, attributes);
    }

    pugi::xml_node new_geom(std::string const& name,
                            std::string const& type,
                            Attributes const& attributes,
                            globs::VectF4 rgba=globs::RED,
                            globs::Vect3 pos={0, 0, 0},
                            vector<float> size={0.005})
    {
        return newElement("site", name, attributes);
    }

    pugi::xml_node new_inertial( Attributes const& attributes, float mass=0.f,
                                 globs::Vect3 pos={0, 0, 0})
    {
        return newElement("inertial", "", attributes);
    }

    std::vector<float> getSize(const std::vector<float>& size,
                               const std::vector<float>& size_max,
                               const std::vector<float>& size_min,
                               const std::vector<float>& default_max,
                               const std::vector<float>& default_min)
    {
        return {};
    }
    pugi::xml_node postprocess_model_xml(string const & xml_str)
    {
        return {};
    }


    bool starts_with(string const& str, string const& regex)
    {
        return str.rfind(regex, 0) == 0;
    }

    void add_prefix(
            pugi::xml_node root,
            std::string const& prefix,
            std::function<bool(std::string)> const& excludes={},
            vector<std::string> const& tags={"default"},
            vector<std::string> attribs={"default"})
    {

        bool default_tag = found<string>(tags, "default");

        if (found<string>(attribs, "default"))
        {
            attribs = globs::MUJOCO_NAMED_ATTRIBUTES;
        }

        GetAllPredicate predicate([&](pugi::xml_node& node){
            for(auto& attrib_name: attribs)
            {
                if((default_tag || found<string>(tags, root.name())) && (!excludes || excludes(node.name())))
                {
                    auto attrib = root.attribute(attrib_name.c_str());
                    if(attrib.value() && !starts_with(attrib.value(), prefix) && (!excludes || excludes(attrib.value())))
                    {
                        attrib.set_value((prefix + attrib.value()).c_str());
                    }
                }
            }
            return false;
        },false);
        root.traverse(predicate);
    }

    struct MatOutput
    {
        pugi::xml_node tex_element, mat_element;
        CustomMaterial custom_material;
        bool used;
    };

    MatOutput add_material(
            pugi::xml_node root,
            const std::string& namingPrefix="",
            CustomMaterial const& customMaterial={})
    {
        return {};
    }

    void recolorCollisionGeoms(pugi::xml_node root, globs::VectF4 rgba)
    {

    }

    void recolorCollisionGeoms(pugi::xml_node root, globs::VectF4 rgba, std::function<bool(pugi::xml_node)> const& excludes)
    {

    }


    bool tag_equal(Element ele, const char* st2)
    {
        return strcmp(ele.name(), st2) == 0;
    }

    bool equal(const char* st1, const char* st2)
    {
        return strcmp(st1, st2) == 0;
    }

    bool tag_in(Element ele, vector<std::string> list_)
    {
        return std::find(begin(list_), end(list_), ele.name()) != end(list_);
    }

    std::string _elementFilter(Element element, Element parent)
    {
        if(parent && tag_equal(parent, "actuator"))
        {
            return "actuators";
        }
        else if (tag_equal(element, "joint"))
        {
            if(!element.attribute("joint").value() && !element.attribute("joint1").value())
            {
                return "joints";
            }
        }
        else if (tag_equal(element, "body"))
        {
            if(!parent || !tag_equal(parent, "body"))
            {
                return "root_body";
            }
            return "bodies";
        }
        else if (tag_equal(element, "site"))
            return "sites";
        else if (tag_in(element, globs::SENSOR_TYPES))
            return "sensors";
        else if (tag_equal(element, "geom"))
        {
            auto group = element.attribute("group").value();
            if( !group || equal(group, "0"))
                return "contact_geoms";
            else if(equal(group, "1"))
                return "visual_geoms";
        }
        else
            return "";
    }


    ElementDict sortElements(pugi::xml_node root,
                             pugi::xml_node parent= {},
                             const std::function<std::string(pugi::xml_node, pugi::xml_node)>& elementFilter={},
                             ElementDict const& _elements_dict={})
    {

        auto filter = elementFilter ? elementFilter : _elementFilter;
        ElementDict elementDict = _elements_dict;

        auto key = filter(root, parent);
        if (!key.empty())
        {
            if(elementDict.find(key) != end(elementDict))
                elementDict[key] = {root};
            else
                elementDict[key].push_back(root);
        }

        for(auto r: root)
        {
            elementDict = sortElements(r, root, elementFilter, _elements_dict);
        }

        return elementDict;
    }

    pugi::xml_node find_parent(pugi::xml_node child)
    {
        return child.parent();
    }

    std::vector<pugi::xml_node> findElements(pugi::xml_node root,
                                             vector<string> const& tags,
                                             StrDict const& attrib={},
                                             bool returnFirst=true)
    {
        GetAllPredicate predicate([&](pugi::xml_node node){
            if(find(begin(tags), end(tags), node.name()) != end(tags))
            {
                for_each(begin(attrib), end(attrib), [&](auto attribute)
                {
                    if(node.attribute(attribute.first.c_str()).value() != attribute.second)
                        return false;
                });
                return true;
            }

            return false;
        }, returnFirst);
        root.traverse(predicate);
        return predicate.nodes;
    }

    pugi::xml_node findElements(pugi::xml_node root,
                                string const& tags,
                                StrDict const& attrib={},
                                bool returnFirst=true)
    {
        return findElements(root, vector<string>{tags}, attrib, returnFirst).front();
    }

//    void saveSimModel(MjSim* sim, string const& fName)
//    {
//        sim->save(fName);
//    }
}


#endif //FASTDRL_MJCF_UTILS_H
