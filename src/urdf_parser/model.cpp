/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <iostream>
#include "urdf_parser/urdf_parser.h"

namespace urdf {

bool parseMaterial(Material &material, tinyxml2::XMLElement *config, bool only_name_is_ok);
bool parseLink(Link &link, tinyxml2::XMLElement *config);
bool parseJoint(Joint &joint, tinyxml2::XMLElement *config);

ModelInterfaceSharedPtr parseURDFFile(const std::string &path)
{
  std::ifstream stream(path.c_str());
  if (!stream)
  {
    std::cerr << ("File " + path + " does not exist").c_str() << std::endl;
    return ModelInterfaceSharedPtr();
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)),
                      std::istreambuf_iterator<char>());
  return urdf::parseURDF(xml_str);
}

bool assignMaterial(const VisualSharedPtr &visual, ModelInterfaceSharedPtr &model, const char *link_name)
{
  if (visual->material_name.empty())
    return true;

  const MaterialSharedPtr &material = model->getMaterial(visual->material_name);
  if (material)
  {
    std::cout << "urdfdom: setting link '" << link_name << "' material to '" << visual->material_name << "'" << std::endl;
    visual->material = material;
  }
  else
  {
    if (visual->material)
    {
      std::cout << "urdfdom: link '" << link_name << "' material '" << visual->material_name << "' defined in Visual." << std::endl;
      model->materials_.insert(make_pair(visual->material->name, visual->material));
    }
    else
    {
      std::cerr << "link '" << link_name << "' material '" << visual->material_name << "' undefined." << std::endl;
      return false;
    }
  }
  return true;
}

ModelInterfaceSharedPtr parseURDF(const std::string &xml_string)
{
  ModelInterfaceSharedPtr model(new ModelInterface);
  model->clear();

  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
  {
    std::cerr << xml_doc.ErrorStr() << std::endl;
    xml_doc.ClearError();
    model.reset();
    return model;
  }

  tinyxml2::XMLElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    std::cerr << "Could not find the 'robot' element in the xml file" << std::endl;
    model.reset();
    return model;
  }

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    std::cerr << "No name given for the robot." << std::endl;
    model.reset();
    return model;
  }
  model->name_ = std::string(name);

  try
  {
    urdf_export_helpers::URDFVersion version(robot_xml->Attribute("version"));
    if (!version.equal(1, 0))
    {
      throw std::runtime_error("Invalid 'version' specified; only version 1.0 is currently supported");
    }
  } catch (const std::runtime_error &err)
  {
    std::cerr << err.what() << std::endl;
    model.reset();
    return model;
  }

  // Get all Material elements
  for (tinyxml2::XMLElement *material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    MaterialSharedPtr material;
    material.reset(new Material);

    try
    {
      parseMaterial(*material, material_xml, false); // material needs to be fully defined here
      if (model->getMaterial(material->name))
      {
        std::cerr << "material '" << material->name.c_str() << "' is not unique." << std::endl;
        material.reset();
        model.reset();
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name, material));
        std::cout << "urdfdom: successfully added a new material '" << material->name.c_str() << "'" << std::endl;
      }
    } catch (ParseError & /*e*/)
    {
      std::cerr << "material xml is not initialized correctly" << std::endl;
      material.reset();
      model.reset();
      return model;
    }
  }

  // Get all Link elements
  for (tinyxml2::XMLElement *link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    LinkSharedPtr link;
    link.reset(new Link);

    try
    {
      parseLink(*link, link_xml);
      if (model->getLink(link->name))
      {
        std::cerr << "link '" << link->name.c_str() << "' is not unique." << std::endl;
        model.reset();
        return model;
      }
      else
      {
        // set link visual(s) material
        std::cout << "urdfdom: setting link '" << link->name.c_str() << "' material" << std::endl;
        if (link->visual)
        {
          assignMaterial(link->visual, model, link->name.c_str());
        }
        for (const auto &visual : link->visual_array)
        {
          assignMaterial(visual, model, link->name.c_str());
        }

        model->links_.insert(make_pair(link->name, link));
        std::cout << "urdfdom: successfully added a new link '" << link->name.c_str() << "'" << std::endl;
      }
    } catch (ParseError & /*e*/)
    {
      std::cerr << "link xml is not initialized correctly" << std::endl;
      model.reset();
      return model;
    }
  }
  if (model->links_.empty())
  {
    std::cerr << "No link elements found in urdf file" << std::endl;
    model.reset();
    return model;
  }

  // Get all Joint elements
  for (tinyxml2::XMLElement *joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    JointSharedPtr joint;
    joint.reset(new Joint);

    if (parseJoint(*joint, joint_xml))
    {
      if (model->getJoint(joint->name))
      {
        std::cerr << "joint '" << joint->name.c_str() << "' is not unique." << std::endl;
        model.reset();
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name, joint));
        std::cout << "urdfdom: successfully added a new joint '" << joint->name.c_str() << "'" << std::endl;
      }
    }
    else
    {
      std::cerr << "joint xml is not initialized correctly" << std::endl;
      model.reset();
      return model;
    }
  }


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try
  {
    model->initTree(parent_link_tree);
  } catch (ParseError &e)
  {
    std::cerr << "Failed to build tree: " << e.what() << std::endl;
    model.reset();
    return model;
  }

  // find the root link
  try
  {
    model->initRoot(parent_link_tree);
  } catch (ParseError &e)
  {
    std::cerr << "Failed to find root link: " << e.what() << std::endl;
    model.reset();
    return model;
  }

  return model;
}

#ifdef ENABLE_URDF_EXPORT

bool exportMaterial(Material &material, tinyxml2::XMLElement *config);
bool exportLink(Link &link, tinyxml2::XMLElement *config);
bool exportJoint(Joint &joint, tinyxml2::XMLElement *config);
tinyxml2::XMLDocument *exportURDF(const ModelInterface &model)
{
  tinyxml2::XMLDocument *doc = new tinyxml2::XMLDocument();

  tinyxml2::XMLElement *robot = new tinyxml2::XMLElement("robot");
  robot->SetAttribute("name", model.name_);
  doc->LinkEndChild(robot);


  for (std::map<std::string, MaterialSharedPtr>::const_iterator m = model.materials_.begin(); m != model.materials_.end(); m++)
  {
    std::cout << "urdfdom: exporting material [" << m->second->name.c_str() << "]\n"
              << std::endl;
    exportMaterial(*(m->second), robot);
  }

  for (std::map<std::string, LinkSharedPtr>::const_iterator l = model.links_.begin(); l != model.links_.end(); l++)
  {
    std::cout << "urdfdom: exporting link [" << l->second->name.c_str() << "]\n"
              << std::endl;
    exportLink(*(l->second), robot);
  }

  for (std::map<std::string, JointSharedPtr>::const_iterator j = model.joints_.begin(); j != model.joints_.end(); j++)
  {
    std::cout << "urdfdom: exporting joint [" << j->second->name.c_str() << "]\n"
              << std::endl;
    exportJoint(*(j->second), robot);
  }

  return doc;
}

tinyxml2::XMLDocument *exportURDF(ModelInterfaceSharedPtr &model)
{
  return exportURDF(*model);
}

#endif

}
