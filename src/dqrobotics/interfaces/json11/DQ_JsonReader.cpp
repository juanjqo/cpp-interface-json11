/**
(C) Copyright 2019 DQ Robotics Developers
<<<<<<< HEAD:src/dqrobotics/interfaces/DQ_JsonReader.cpp

This file is part of DQ Robotics.

=======
This file is part of DQ Robotics.
>>>>>>> 9970128a05d15075abbd2e3b35b449fa0ac422b4:src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
<<<<<<< HEAD:src/dqrobotics/interfaces/DQ_JsonReader.cpp

=======
>>>>>>> 9970128a05d15075abbd2e3b35b449fa0ac422b4:src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
<<<<<<< HEAD:src/dqrobotics/interfaces/DQ_JsonReader.cpp

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
*/

#include <dqrobotics/interfaces/DQ_JsonReader.h>
#include <dqrobotics/utils/DQ_Math.h>

#include <json11.hpp>

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<exception>
#include<vector>

namespace DQ_robotics
{

json11::Json parse_json(const std::string& file)
{
    std::string error;
    std::ifstream f(file); //taking file as inputstream
    std::string str;
    if(f) {
        std::ostringstream ss;
        ss << f.rdbuf(); // reading data
        str = ss.str();
    }
    json11::Json parsed_json = json11::Json::parse(str, error);
    if(error != "")
        throw std::runtime_error("Json parse error: " + error);
    return parsed_json;
}

VectorXd get_eigen_vectorxd_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    VectorXd eigen_vector(json_vector.size());
    for(int i=0;i<json_vector.size();i++)
    {
        eigen_vector(i) = json_vector[i].number_value();
    }
    return eigen_vector;
}

VectorXi get_eigen_vectorxi_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    VectorXi eigen_vector(json_vector.size());
    for(int i=0;i<json_vector.size();i++)
    {
        eigen_vector(i) = json_vector[i].int_value();
    }
    return eigen_vector;
}

DQ_SerialManipulator DQ_JsonReader::_get_serial_manipulator_from_json(const std::string &file)
{
    json11::Json parsed_json = parse_json(file);

    //Type
    std::string type = parsed_json["type"].string_value();
    if(type != "DQ_SerialManipulator")
        throw std::runtime_error("get_serial_manipulator_from_json is only compatible with DQ_SerialManipulator and not " + type);

    //ANGLE MODE
    std::string angle_mode_str = parsed_json["angle_mode"].string_value();
    bool angle_mode_degree;
    if(angle_mode_str == "degree")
        angle_mode_degree = true;
    else if(angle_mode_str == "rad")
        angle_mode_degree = false;
    else
        throw std::runtime_error("Unable to decode angle_mode");

    std::string convention = parsed_json["convention"].string_value();

    //THETA
    VectorXd theta_vec = get_eigen_vectorxd_from_json_vector(parsed_json["theta"].array_items());
    if(angle_mode_degree)
        theta_vec = deg2rad(theta_vec);
    //D
    VectorXd d_vec = get_eigen_vectorxd_from_json_vector(parsed_json["d"].array_items());
    //A
    VectorXd a_vec = get_eigen_vectorxd_from_json_vector(parsed_json["a"].array_items());
    //ALPHA
    VectorXd alpha_vec = get_eigen_vectorxd_from_json_vector(parsed_json["alpha"].array_items());
    if(angle_mode_degree)
        alpha_vec = deg2rad(alpha_vec);

    MatrixXd dh_matrix(4,theta_vec.size());
    dh_matrix << theta_vec.transpose(),
            d_vec.transpose(),
            a_vec.transpose(),
            alpha_vec.transpose();

    //LOWER LIMIT
    VectorXd lower_vec = get_eigen_vectorxd_from_json_vector(parsed_json["lower_q_limit"].array_items());
    if(angle_mode_degree)
        lower_vec = deg2rad(lower_vec);
    //UPPER LIMIT
    VectorXd upper_vec = get_eigen_vectorxd_from_json_vector(parsed_json["upper_q_limit"].array_items());
    if(angle_mode_degree)
        upper_vec = deg2rad(upper_vec);

    DQ_SerialManipulator serial_manipulator(dh_matrix, convention);
    serial_manipulator.set_lower_q_limit(lower_vec);
    serial_manipulator.set_upper_q_limit(upper_vec);

    return serial_manipulator;
}

DQ_SerialManipulatorDH DQ_JsonReader::_get_serial_manipulator_dh_from_json(const std::string &file)
{
    json11::Json parsed_json = parse_json(file);

    //Type
    std::string type = parsed_json["type"].string_value();
    if(type != "DQ_SerialManipulatorDH")
        throw std::runtime_error("get_serial_manipulator_from_json is only compatible with DQ_SerialManipulator and not " + type);

    //ANGLE MODE
    std::string angle_mode_str = parsed_json["angle_mode"].string_value();
    bool angle_mode_degree;
    if(angle_mode_str == "degree")
        angle_mode_degree = true;
    else if(angle_mode_str == "rad")
        angle_mode_degree = false;
    else
        throw std::runtime_error("Unable to decode angle_mode");

    std::string convention = parsed_json["convention"].string_value();

    //THETA
    VectorXd theta_vec = get_eigen_vectorxd_from_json_vector(parsed_json["theta"].array_items());
    if(angle_mode_degree)
        theta_vec = deg2rad(theta_vec);
    //D
    VectorXd d_vec = get_eigen_vectorxd_from_json_vector(parsed_json["d"].array_items());
    //A
    VectorXd a_vec = get_eigen_vectorxd_from_json_vector(parsed_json["a"].array_items());
    //ALPHA
    VectorXd alpha_vec = get_eigen_vectorxd_from_json_vector(parsed_json["alpha"].array_items());
    if(angle_mode_degree)
        alpha_vec = deg2rad(alpha_vec);
    //Joint Types
    VectorXi joint_types = get_eigen_vectorxi_from_json_vector(parsed_json["joint_types"].array_items());

    MatrixXd dh_matrix(5,theta_vec.size());
    dh_matrix << theta_vec.transpose(),
            d_vec.transpose(),
            a_vec.transpose(),
            alpha_vec.transpose(),
            joint_types.cast<double>().transpose();

    //LOWER LIMIT
    VectorXd lower_vec = get_eigen_vectorxd_from_json_vector(parsed_json["lower_q_limit"].array_items());
    if(angle_mode_degree)
        lower_vec = deg2rad(lower_vec);
    //UPPER LIMIT
    VectorXd upper_vec = get_eigen_vectorxd_from_json_vector(parsed_json["upper_q_limit"].array_items());
    if(angle_mode_degree)
        upper_vec = deg2rad(upper_vec);

    DQ_SerialManipulatorDH serial_manipulator_dh(dh_matrix);
    serial_manipulator_dh.set_lower_q_limit(lower_vec);
    serial_manipulator_dh.set_upper_q_limit(upper_vec);

    return serial_manipulator_dh;
}

template<typename T>
T DQ_JsonReader::get_from_json(const std::string &)
{
    throw std::runtime_error("get_from_json not defined for chosen type.\n"
                             "It is currently defined for:\n"
                             "DQ_SerialKinematics\n"
                             "DQ_SerialKinematicsDH\n");
}

template <>
DQ_SerialManipulator DQ_JsonReader::get_from_json<DQ_SerialManipulator>(const std::string& file)
{
    return _get_serial_manipulator_from_json(file);
}

template <>
DQ_SerialManipulatorDH DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(const std::string& file)
{
    return _get_serial_manipulator_dh_from_json(file);
}

}
