//
// Created by dewe on 4/24/21.
//

#pragma once

#include "string"
#include "pugixml.hpp"
#include "vector"
#include "mujoco.h"
#include <functional>
#include <filesystem>
#include "opencv4/opencv2/opencv.hpp"
#include "globals.h"
#include <type_traits>


namespace mjcfUtils
{

    using std::vector;
    using std::string;
    using Element = pugi::xml_node;
    using ElementDict = std::unordered_map<std::string, vector<Element>>;
    using StrDict = std::unordered_map<std::string, std::string>;
    using Attributes = StrDict;
    struct GetAllPredicate : pugi::xml_tree_walker
    {
        std::vector<pugi::xml_node> nodes;
        std::function<bool(pugi::xml_node&)> predicate;
        bool return_first{false};

        explicit GetAllPredicate( std::function<bool(pugi::xml_node&)> predicate, bool return_first): predicate(std::move(predicate)), return_first(return_first)
        {

        }

        bool for_each(pugi::xml_node& node) override
        {
            if(predicate(node))
            {
                nodes.push_back(node);
                if(return_first)
                    return false;
            }

            return true; // continue traversal
        }
    };

    string arrayToString_(at::Tensor const& tensor);

    std::string xmlPathCompletion(std::filesystem::path const& xmlPath);
    struct CustomMaterial;

    template<typename T>
    auto stringToArray( string const& inp) {
        vector<T> vec;
        std::for_each(begin(inp), end(inp), [&vec](char c)
           {
                if (c != ' ')
                    vec.push_back(static_cast<T>(c));
           }
        );
        return vec;
    }

    template<typename T>
    concept List = std::is_aggregate<T>::value;
    template<List ListType>
    auto convertToString( ListType const& vec)
    {
        std::stringstream ss;
        std::for_each(begin(vec), end(vec), [&vec, &ss](auto c)
                      {
                          ss << std::to_string(c);
                      }
        );
        return ss.str();
    }

    template<typename T>
    concept Scalar = std::is_scalar<T>::value;
    template<Scalar T>
    auto convertToString( T scalar )
    {
        return std::to_string(scalar);
    }

    void setAlpha(pugi::xml_node node, float alpha);
    pugi::xml_node newElement(string const& tag,
                              string const& name,
                              Attributes const& attributes);

    pugi::xml_node new_joint(string const& name, Attributes const& attributes);
    pugi::xml_node new_actuator(string const& name, string const& joint,  Attributes const& attributes, string const& act_type);

    pugi::xml_node new_site(std::string const& name,
                            Attributes const& attributes,
                            vector<float> size,
                            globs::Vect3 pos,
                            int group);

    pugi::xml_node new_body(std::string const& name,
                            Attributes const& attributes,
                            globs::Vect3 pos);

    pugi::xml_node new_geom(std::string const& name,
                            std::string const& type,
                            Attributes const& attributes,
                            globs::VectF4 rgba,
                            globs::Vect3 pos,
                            vector<float> size);

    pugi::xml_node new_inertial( Attributes const& attributes, float mass,globs::Vect3 pos);

    std::vector<float> getSize(const std::vector<float>& size,
                               const std::vector<float>& size_max,
                               const std::vector<float>& size_min,
                               const std::vector<float>& default_max,
                               const std::vector<float>& default_min);

    pugi::xml_node postprocess_model_xml(string const & xml_str);

    template<typename KeyType, typename ValueType>
    std::unordered_map<KeyType, ValueType> add_to_dict(std::unordered_map<KeyType, ValueType > const& dic,
                                    bool fill_in_defaults=true,
                                    std::optional<ValueType> const& default_value=std::nullopt,
                                    std::unordered_map<KeyType,ValueType> const& kwargs={})
    {
            return {};
    }

    template<class T>
    bool found(vector<T> const& _list, T val)
    {
        return find(begin(_list), end(_list), val) != end(_list);
    }

    template<class T, class V>
    bool found(std::unordered_map<T, V> const& _list, T val)
    {
        return _list.find(val) != end(_list);
    }

    bool starts_with(string const& str, string const& regex);

    void add_prefix(
            pugi::xml_node root,
            std::string const& prefix,
            std::function<bool(std::string)> const& excludes,
            vector<std::string> const& tags,
            vector<std::string> attribs);

    struct MatOutput;

    MatOutput add_material(
            pugi::xml_node root,
            const std::string& namingPrefix,
            CustomMaterial const& customMaterial);

    void recolorCollisionGeoms(pugi::xml_node root, globs::VectF4 rgba);

    void recolorCollisionGeoms(pugi::xml_node root, globs::VectF4 rgba, std::function<bool(pugi::xml_node)> const& excludes);

    bool tag_equal(Element ele, const char* st2);

    bool equal(const char* st1, const char* st2);

    bool tag_in(Element ele, vector<std::string> list_);

    std::string _elementFilter(Element element, Element parent);

    ElementDict sortElements(pugi::xml_node root,
                             pugi::xml_node parent,
                                const std::function<std::string(pugi::xml_node, pugi::xml_node)>& elementFilter,
                                ElementDict const& _elements_dict);

    pugi::xml_node find_parent(pugi::xml_node child);


    std::vector<pugi::xml_node> findElements(pugi::xml_node root,
                                                             vector<string> const& tags,
                                                             StrDict const& attrib,
                                                             bool returnFirst);

    pugi::xml_node findElements(pugi::xml_node root,
                                             string const& tags,
                                             StrDict const& attrib,
                                             bool returnFirst);

    void saveSimModel(struct MjSim* sim, string const& fName);

}
