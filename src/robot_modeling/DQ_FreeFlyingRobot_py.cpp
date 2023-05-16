/**
(C) Copyright 2023 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
    1. Bruno Vilhena Adorno (adorno@ieee.org)
       Responsible for the original implementation in file DQ_FreeFlyingRobot.m
       https://github.com/dqrobotics/matlab/pull/66/commits

    2. Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
       Created this file.
*/

#include "../dqrobotics_module.h"

void init_DQ_FreeFlyingRobot_py(py::module& m)
{
    /***************************************************
    *  DQ FreeFlyingRobot
    * **************************************************/
    py::class_<
            DQ_FreeFlyingRobot,
            std::shared_ptr<DQ_FreeFlyingRobot>,
            DQ_Kinematics
            > dqfreeflyingrobot_py(m, "DQ_FreeFlyingRobot");

    dqfreeflyingrobot_py.def(py::init());

    ///Methods
    //Concrete
    dqfreeflyingrobot_py.def("fkm",(DQ (DQ_FreeFlyingRobot::*)(const DQ&) const)&DQ_FreeFlyingRobot::fkm,"Gets the the free-flying robot pose.");
    dqfreeflyingrobot_py.def("pose_jacobian",(MatrixXd (DQ_FreeFlyingRobot::*)(const DQ&) const)&DQ_FreeFlyingRobot::pose_jacobian,"Gets the free-flying robot pose Jacobian.");
    dqfreeflyingrobot_py.def("pose_jacobian_derivative",(MatrixXd (DQ_FreeFlyingRobot::*)(const DQ&) const)&DQ_FreeFlyingRobot::pose_jacobian_derivative,"Gets the free-flying robot pose Jacobian derivative.");


    //Overrides from DQ_Kinematics
    dqfreeflyingrobot_py.def("fkm",                         (DQ (DQ_FreeFlyingRobot::*)(const VectorXd&) const)&DQ_FreeFlyingRobot::fkm,"Gets the fkm.");
    dqfreeflyingrobot_py.def("fkm",                         (DQ (DQ_FreeFlyingRobot::*)(const VectorXd&,const int&) const)&DQ_FreeFlyingRobot::fkm,"Gets the fkm.");
    dqfreeflyingrobot_py.def("pose_jacobian",               (MatrixXd (DQ_FreeFlyingRobot::*)(const VectorXd&, const int&) const)&DQ_FreeFlyingRobot::pose_jacobian,"Returns the pose Jacobian.");
    dqfreeflyingrobot_py.def("pose_jacobian_derivative",    (MatrixXd (DQ_FreeFlyingRobot::*)(const VectorXd&, const VectorXd&, const int&) const)
                                                               &DQ_FreeFlyingRobot::pose_jacobian_derivative,"Returns the pose Jacobian derivative.");
}
