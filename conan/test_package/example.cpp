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

#include "urdf_parser/urdf_parser.h"
#include <fstream>
#include <iostream>

using namespace urdf;

std::string model = R"(
<robot name="XYZSarm">
    <link name="b0">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="1"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
        </inertial>
        <visual>
            <origin rpy="0. 0. 0." xyz=".1 .2 .3"/>
            <geometry>
                <mesh filename="test_mesh1.dae"/>
            </geometry>
        </visual>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="test_mesh2.dae"/>
            </geometry>
        </visual>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
        </visual>
    </link>
    <link name="b1">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0.5 0"/>
            <mass value="5."/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
        </inertial>
    </link>
    <link name="b2">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0.5 0"/>
            <mass value="2."/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
        </inertial>
    </link>
    <link name="b3">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0.5 0"/>
            <mass value="1.5"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
        </inertial>
    </link>
    <link name="b4">
        <inertial>
            <origin rpy="0 0 0" xyz="0.5 0 0"/>
            <mass value="1"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
        </inertial>
    </link>
    <joint name="j0" type="revolute">
        <parent link="b0"/>
        <child link="b1"/>
        <origin rpy="0 0 0" xyz="0 1 0"/>
        <axis xyz="1 0 0"/>
        <limit lower="-1" upper="1" velocity="10" effort="50"/>
    </joint>
    <joint name="j1" type="revolute">
        <parent link="b1"/>
        <child link="b2"/>
        <origin rpy="0 0 0" xyz="0 1 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" velocity="10" effort="50"/>
    </joint>
    <joint name="j2" type="revolute">
        <parent link="b2"/>
        <child link="b3"/>
        <origin rpy="0 0 0" xyz="0 1 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" velocity="10" effort="50"/>
    </joint>
    <joint name="j3" type="continuous">
        <parent link="b1"/>
        <child link="b4"/>
        <origin rpy="1. 0 0" xyz="1 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
</robot>
)";

void printTree(LinkConstSharedPtr link, int level = 0) {
  level += 2;
  int count = 0;
  for (std::vector<LinkSharedPtr>::const_iterator child =
           link->child_links.begin();
       child != link->child_links.end(); child++) {
    if (*child) {
      for (int j = 0; j < level; j++)
        std::cout << "  "; // indent
      std::cout << "child(" << (count++) + 1 << "):  " << (*child)->name
                << std::endl;
      // first grandchild
      printTree(*child, level);
    } else {
      for (int j = 0; j < level; j++)
        std::cout << " "; // indent
      std::cout << "root link: " << link->name << " has a null child!" << *child
                << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  ModelInterfaceSharedPtr robot = parseURDF(model);
  if (!robot) {
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  std::cout << "robot name is: " << robot->getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------"
            << std::endl;
  // get root link
  LinkConstSharedPtr root_link = robot->getRoot();
  if (!root_link)
    return -1;

  std::cout << "root Link: " << root_link->name << " has "
            << root_link->child_links.size() << " child(ren)" << std::endl;

  // print entire tree
  printTree(root_link);
  return 0;
}

