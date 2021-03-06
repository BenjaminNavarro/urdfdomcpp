/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <iostream>
#include <urdf_model/joint.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf {

bool parsePose(Pose &pose, tinyxml2::XMLElement *xml);

bool parseJointDynamics(JointDynamics &jd, tinyxml2::XMLElement *config)
{
  jd.clear();

  // Get joint damping
  const char *damping_str = config->Attribute("damping");
  if (damping_str == NULL)
  {
    std::cout << "urdfdom.joint_dynamics: no damping, defaults to 0" << std::endl;
    jd.damping = 0;
  }
  else
  {
    try
    {
      jd.damping = strToDouble(damping_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "damping value (" << damping_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get joint friction
  const char *friction_str = config->Attribute("friction");
  if (friction_str == NULL)
  {
    std::cout << "urdfdom.joint_dynamics: no friction, defaults to 0" << std::endl;
    jd.friction = 0;
  }
  else
  {
    try
    {
      jd.friction = strToDouble(friction_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "friction value (" << friction_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    std::cerr << "joint dynamics element specified with no damping and no friction" << std::endl;
    return false;
  }
  else
  {
    std::cout << "urdfdom.joint_dynamics: damping " << jd.damping << " and friction " << jd.friction << std::endl;
    return true;
  }
}

bool parseJointLimits(JointLimits &jl, tinyxml2::XMLElement *config)
{
  jl.clear();

  // Get lower joint limit
  const char *lower_str = config->Attribute("lower");
  if (lower_str == NULL)
  {
    std::cout << "urdfdom.joint_limit: no lower, defaults to 0" << std::endl;
    jl.lower = 0;
  }
  else
  {
    try
    {
      jl.lower = strToDouble(lower_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "lower value (" << lower_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get upper joint limit
  const char *upper_str = config->Attribute("upper");
  if (upper_str == NULL)
  {
    std::cout << "urdfdom.joint_limit: no upper, , defaults to 0" << std::endl;
    jl.upper = 0;
  }
  else
  {
    try
    {
      jl.upper = strToDouble(upper_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "upper value (" << upper_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get joint effort limit
  const char *effort_str = config->Attribute("effort");
  if (effort_str == NULL)
  {
    std::cerr << "joint limit: no effort" << std::endl;
    return false;
  }
  else
  {
    try
    {
      jl.effort = strToDouble(effort_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "effort value (" << effort_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get joint velocity limit
  const char *velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL)
  {
    std::cerr << "joint limit: no velocity" << std::endl;
    return false;
  }
  else
  {
    try
    {
      jl.velocity = strToDouble(velocity_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "velocity value (" << velocity_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  return true;
}

bool parseJointSafety(JointSafety &js, tinyxml2::XMLElement *config)
{
  js.clear();

  // Get soft_lower_limit joint limit
  const char *soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    std::cout << "urdfdom.joint_safety: no soft_lower_limit, using default value" << std::endl;
    js.soft_lower_limit = 0;
  }
  else
  {
    try
    {
      js.soft_lower_limit = strToDouble(soft_lower_limit_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "soft_lower_limit value (" << soft_lower_limit_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get soft_upper_limit joint limit
  const char *soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    std::cout << "urdfdom.joint_safety: no soft_upper_limit, using default value" << std::endl;
    js.soft_upper_limit = 0;
  }
  else
  {
    try
    {
      js.soft_upper_limit = strToDouble(soft_upper_limit_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "soft_upper_limit value (" << soft_upper_limit_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get k_position_ safety "position" gain - not exactly position gain
  const char *k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    std::cout << "urdfdom.joint_safety: no k_position, using default value" << std::endl;
    js.k_position = 0;
  }
  else
  {
    try
    {
      js.k_position = strToDouble(k_position_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "k_position value (" << k_position_str << ") is not a valid float" << std::endl;
      return false;
    }
  }
  // Get k_velocity_ safety velocity gain
  const char *k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    std::cerr << "joint safety: no k_velocity" << std::endl;
    return false;
  }
  else
  {
    try
    {
      js.k_velocity = strToDouble(k_velocity_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "k_velocity value (" << k_velocity_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  return true;
}

bool parseJointCalibration(JointCalibration &jc, tinyxml2::XMLElement *config)
{
  jc.clear();

  // Get rising edge position
  const char *rising_position_str = config->Attribute("rising");
  if (rising_position_str == NULL)
  {
    std::cout << "urdfdom.joint_calibration: no rising, using default value" << std::endl;
    jc.rising.reset();
  }
  else
  {
    try
    {
      jc.rising.reset(new double(strToDouble(rising_position_str)));
    } catch (std::runtime_error &)
    {
      std::cerr << "rising value (" << rising_position_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  // Get falling edge position
  const char *falling_position_str = config->Attribute("falling");
  if (falling_position_str == NULL)
  {
    std::cout << "urdfdom.joint_calibration: no falling, using default value" << std::endl;
    jc.falling.reset();
  }
  else
  {
    try
    {
      jc.falling.reset(new double(strToDouble(falling_position_str)));
    } catch (std::runtime_error &)
    {
      std::cerr << "falling value (" << falling_position_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  return true;
}

bool parseJointMimic(JointMimic &jm, tinyxml2::XMLElement *config)
{
  jm.clear();

  // Get name of joint to mimic
  const char *joint_name_str = config->Attribute("joint");

  if (joint_name_str == NULL)
  {
    std::cerr << "joint mimic: no mimic joint specified" << std::endl;
    return false;
  }
  else
    jm.joint_name = joint_name_str;

  // Get mimic multiplier
  const char *multiplier_str = config->Attribute("multiplier");

  if (multiplier_str == NULL)
  {
    std::cout << "urdfdom.joint_mimic: no multiplier, using default value of 1" << std::endl;
    jm.multiplier = 1;
  }
  else
  {
    try
    {
      jm.multiplier = strToDouble(multiplier_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "multiplier value (" << multiplier_str << ") is not a valid float" << std::endl;
      return false;
    }
  }


  // Get mimic offset
  const char *offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    std::cout << "urdfdom.joint_mimic: no offset, using default value of 0" << std::endl;
    jm.offset = 0;
  }
  else
  {
    try
    {
      jm.offset = strToDouble(offset_str);
    } catch (std::runtime_error &)
    {
      std::cerr << "offset value (" << offset_str << ") is not a valid float" << std::endl;
      return false;
    }
  }

  return true;
}

bool parseJoint(Joint &joint, tinyxml2::XMLElement *config)
{
  joint.clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    std::cerr << "unnamed joint found" << std::endl;
    return false;
  }
  joint.name = name;

  // Get transform from Parent Link to Joint Frame
  tinyxml2::XMLElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    std::cout << "urdfdom: Joint [" << joint.name.c_str() << "] missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform)." << std::endl;
    joint.parent_to_joint_origin_transform.clear();
  }
  else
  {
    if (!parsePose(joint.parent_to_joint_origin_transform, origin_xml))
    {
      joint.parent_to_joint_origin_transform.clear();
      std::cerr << "Malformed parent origin element for joint [" << joint.name.c_str() << "]" << std::endl;
      return false;
    }
  }

  // Get Parent Link
  tinyxml2::XMLElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      std::cout << "no parent link name specified for Joint link [" << joint.name.c_str() << "]. this might be the root?" << std::endl;
    }
    else
    {
      joint.parent_link_name = std::string(pname);
    }
  }

  // Get Child Link
  tinyxml2::XMLElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      std::cout << "no child link name specified for Joint link [" << joint.name.c_str() << "]." << std::endl;
    }
    else
    {
      joint.child_link_name = std::string(pname);
    }
  }

  // Get Joint type
  const char *type_char = config->Attribute("type");
  if (!type_char)
  {
    std::cerr << "joint [" << joint.name.c_str() << "] has no type, check to see if it's a reference." << std::endl;
    return false;
  }

  std::string type_str = type_char;
  if (type_str == "planar")
    joint.type = Joint::PLANAR;
  else if (type_str == "floating")
    joint.type = Joint::FLOATING;
  else if (type_str == "revolute")
    joint.type = Joint::REVOLUTE;
  else if (type_str == "continuous")
    joint.type = Joint::CONTINUOUS;
  else if (type_str == "prismatic")
    joint.type = Joint::PRISMATIC;
  else if (type_str == "fixed")
    joint.type = Joint::FIXED;
  else
  {
    std::cerr << "Joint [" << joint.name << "] has no known type [" << type_str << "]" << std::endl;
    return false;
  }

  // Get Joint Axis
  if (joint.type != Joint::FLOATING && joint.type != Joint::FIXED)
  {
    // axis
    tinyxml2::XMLElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml)
    {
      std::cout << "urdfdom: no axis elemement for Joint link [" << joint.name.c_str() << "], defaulting to (1,0,0) axis" << std::endl;
      joint.axis = Vector3(1.0, 0.0, 0.0);
    }
    else
    {
      if (axis_xml->Attribute("xyz"))
      {
        try
        {
          joint.axis.init(axis_xml->Attribute("xyz"));
        } catch (ParseError &e)
        {
          joint.axis.clear();
          std::cerr << "Malformed axis element for joint [" << joint.name << "]: " << e.what() << std::endl;
          return false;
        }
      }
    }
  }

  // Get limit
  tinyxml2::XMLElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    joint.limits.reset(new JointLimits());
    if (!parseJointLimits(*joint.limits, limit_xml))
    {
      std::cerr << "Could not parse limit element for joint [" << joint.name.c_str() << "]" << std::endl;
      joint.limits.reset();
      return false;
    }
  }
  else if (joint.type == Joint::REVOLUTE)
  {
    std::cerr << "Joint [" << joint.name.c_str() << "] is of type REVOLUTE but it does not specify limits" << std::endl;
    return false;
  }
  else if (joint.type == Joint::PRISMATIC)
  {
    std::cerr << "Joint [" << joint.name.c_str() << "] is of type PRISMATIC without limits" << std::endl;
    return false;
  }

  // Get safety
  tinyxml2::XMLElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    joint.safety.reset(new JointSafety());
    if (!parseJointSafety(*joint.safety, safety_xml))
    {
      std::cerr << "Could not parse safety element for joint [" << joint.name.c_str() << "]" << std::endl;
      joint.safety.reset();
      return false;
    }
  }

  // Get calibration
  tinyxml2::XMLElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    joint.calibration.reset(new JointCalibration());
    if (!parseJointCalibration(*joint.calibration, calibration_xml))
    {
      std::cerr << "Could not parse calibration element for joint  [" << joint.name.c_str() << "]" << std::endl;
      joint.calibration.reset();
      return false;
    }
  }

  // Get Joint Mimic
  tinyxml2::XMLElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    joint.mimic.reset(new JointMimic());
    if (!parseJointMimic(*joint.mimic, mimic_xml))
    {
      std::cerr << "Could not parse mimic element for joint  [" << joint.name.c_str() << "]" << std::endl;
      joint.mimic.reset();
      return false;
    }
  }

  // Get Dynamics
  tinyxml2::XMLElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    joint.dynamics.reset(new JointDynamics());
    if (!parseJointDynamics(*joint.dynamics, prop_xml))
    {
      std::cerr << "Could not parse joint_dynamics element for joint [" << joint.name.c_str() << "]" << std::endl;
      joint.dynamics.reset();
      return false;
    }
  }

  return true;
}

#ifdef ENABLE_URDF_EXPORT

/* exports */
bool exportPose(Pose &pose, tinyxml2::XMLElement *xml);

bool exportJointDynamics(JointDynamics &jd, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement *dynamics_xml = new tinyxml2::XMLElement("dynamics");
  dynamics_xml->SetAttribute("damping", urdf_export_helpers::values2str(jd.damping));
  dynamics_xml->SetAttribute("friction", urdf_export_helpers::values2str(jd.friction));
  xml->LinkEndChild(dynamics_xml);
  return true;
}

bool exportJointLimits(JointLimits &jl, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement *limit_xml = new tinyxml2::XMLElement("limit");
  limit_xml->SetAttribute("effort", urdf_export_helpers::values2str(jl.effort));
  limit_xml->SetAttribute("velocity", urdf_export_helpers::values2str(jl.velocity));
  limit_xml->SetAttribute("lower", urdf_export_helpers::values2str(jl.lower));
  limit_xml->SetAttribute("upper", urdf_export_helpers::values2str(jl.upper));
  xml->LinkEndChild(limit_xml);
  return true;
}

bool exportJointSafety(JointSafety &js, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement *safety_xml = new tinyxml2::XMLElement("safety_controller");
  safety_xml->SetAttribute("k_position", urdf_export_helpers::values2str(js.k_position));
  safety_xml->SetAttribute("k_velocity", urdf_export_helpers::values2str(js.k_velocity));
  safety_xml->SetAttribute("soft_lower_limit", urdf_export_helpers::values2str(js.soft_lower_limit));
  safety_xml->SetAttribute("soft_upper_limit", urdf_export_helpers::values2str(js.soft_upper_limit));
  xml->LinkEndChild(safety_xml);
  return true;
}

bool exportJointCalibration(JointCalibration &jc, tinyxml2::XMLElement *xml)
{
  if (jc.falling || jc.rising)
  {
    tinyxml2::XMLElement *calibration_xml = new tinyxml2::XMLElement("calibration");
    if (jc.falling)
      calibration_xml->SetAttribute("falling", urdf_export_helpers::values2str(*jc.falling));
    if (jc.rising)
      calibration_xml->SetAttribute("rising", urdf_export_helpers::values2str(*jc.rising));
    //calibration_xml->SetAttribute("reference_position", urdf_export_helpers::values2str(jc.reference_position) );
    xml->LinkEndChild(calibration_xml);
  }
  return true;
}

bool exportJointMimic(JointMimic &jm, tinyxml2::XMLElement *xml)
{
  if (!jm.joint_name.empty())
  {
    tinyxml2::XMLElement *mimic_xml = new tinyxml2::XMLElement("mimic");
    mimic_xml->SetAttribute("offset", urdf_export_helpers::values2str(jm.offset));
    mimic_xml->SetAttribute("multiplier", urdf_export_helpers::values2str(jm.multiplier));
    mimic_xml->SetAttribute("joint", jm.joint_name);
    xml->LinkEndChild(mimic_xml);
  }
  return true;
}

bool exportJoint(Joint &joint, tinyxml2::XMLElement *xml)
{
  tinyxml2::XMLElement *joint_xml = new tinyxml2::XMLElement("joint");
  joint_xml->SetAttribute("name", joint.name);
  if (joint.type == urdf::Joint::PLANAR)
    joint_xml->SetAttribute("type", "planar");
  else if (joint.type == urdf::Joint::FLOATING)
    joint_xml->SetAttribute("type", "floating");
  else if (joint.type == urdf::Joint::REVOLUTE)
    joint_xml->SetAttribute("type", "revolute");
  else if (joint.type == urdf::Joint::CONTINUOUS)
    joint_xml->SetAttribute("type", "continuous");
  else if (joint.type == urdf::Joint::PRISMATIC)
    joint_xml->SetAttribute("type", "prismatic");
  else if (joint.type == urdf::Joint::FIXED)
    joint_xml->SetAttribute("type", "fixed");
  else
    std::cerr << "ERROR:  Joint [" << joint.name.c_str(), joint.type << "] type [%d] is not a defined type.\n"
                                                                     << std::endl;

  // origin
  exportPose(joint.parent_to_joint_origin_transform, joint_xml);

  // axis
  tinyxml2::XMLElement *axis_xml = new tinyxml2::XMLElement("axis");
  axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(joint.axis));
  joint_xml->LinkEndChild(axis_xml);

  // parent
  tinyxml2::XMLElement *parent_xml = new tinyxml2::XMLElement("parent");
  parent_xml->SetAttribute("link", joint.parent_link_name);
  joint_xml->LinkEndChild(parent_xml);

  // child
  tinyxml2::XMLElement *child_xml = new tinyxml2::XMLElement("child");
  child_xml->SetAttribute("link", joint.child_link_name);
  joint_xml->LinkEndChild(child_xml);

  if (joint.dynamics)
    exportJointDynamics(*(joint.dynamics), joint_xml);
  if (joint.limits)
    exportJointLimits(*(joint.limits), joint_xml);
  if (joint.safety)
    exportJointSafety(*(joint.safety), joint_xml);
  if (joint.calibration)
    exportJointCalibration(*(joint.calibration), joint_xml);
  if (joint.mimic)
    exportJointMimic(*(joint.mimic), joint_xml);

  xml->LinkEndChild(joint_xml);
  return true;
}

#endif

}
