#pragma once
/**
(C) Copyright 2019-2022 DQ Robotics Developers

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
- Murilo M. Marinho        (murilomarinho@ieee.org)
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

class DQ_JsonReader
{
private:
    static DQ_SerialManipulatorDH _get_serial_manipulator_dh_from_json(const std::string& file);
    static DQ_SerialManipulatorMDH _get_serial_manipulator_mdh_from_json(const std::string& file);
    static DQ_SerialManipulatorDenso _get_serial_manipulator_denso_from_json(const std::string& file);
public:
    template <typename T>
    static T get_from_json(const std::string& file);
};


}
