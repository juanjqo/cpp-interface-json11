/**
(C) Copyright 2019-2023 DQ Robotics Developers

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

#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
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

static json11::Json parse_json(const std::string& file)
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

static VectorXd get_eigen_vectorxd_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    VectorXd eigen_vector(json_vector.size());
    for(unsigned long i=0;i<json_vector.size();i++)
    {
        eigen_vector(i) = json_vector[i].number_value();
    }
    return eigen_vector;
}

static VectorXi get_eigen_vectorxi_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    VectorXi eigen_vector(json_vector.size());
    for(unsigned long i=0;i<json_vector.size();i++)
    {
        eigen_vector(i) = json_vector[i].int_value();
    }
    return eigen_vector;
}

static DQ get_unit_dq_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    const DQ json_dq(get_eigen_vectorxd_from_json_vector(json_vector));
    const DQ normalized_dq = normalize(json_dq);
    //Check if the DQ is almost unit
    for(int i=0;i<8;i++)
    {
        if(fabs(normalized_dq.q(i)-json_dq.q(i))>DQ_threshold*2.0)
        {
            throw std::runtime_error("DQ_JsonReader::get_unit_dq_from_json_vector::DQ not almost unit ("+std::to_string(DQ_threshold*2.0)+").");
        }
    }
    //If not already unit, normalize it
    if(!is_unit(json_dq))
        return normalized_dq;

    return json_dq;
}

static void initialize_kinematics_commons(DQ_Kinematics* kinematics,
                                   const json11::Json& parsed_json)
{
    //Reference frame
    DQ reference_frame(get_unit_dq_from_json_vector(parsed_json["reference_frame"].array_items()));

    kinematics->set_reference_frame(reference_frame);
}

static VectorXd deg2rad_with_mask(const VectorXd& v, const VectorXi& mask)
{
    if(v.size()!=mask.size())
        throw std::runtime_error("DQ_JsonReader::deg2rad_with_mask::Invalid mask size.");

    VectorXd v_with_mask(v);
    for(int i=0;i<v.size();i++)
    {
        if(mask(i)==0)
            v_with_mask(i)=deg2rad(v(i));
    }
    return v_with_mask;
}

static void initialize_serial_manipulator_commons(DQ_SerialManipulator* serial_manipulator,
                                           const json11::Json& parsed_json,
                                           const bool& angle_mode_degree,
                                           const VectorXi& angle_mask)
{
    //LOWER LIMIT
    VectorXd lower_vec = get_eigen_vectorxd_from_json_vector(parsed_json["lower_q_limit"].array_items());
    if(angle_mode_degree)
        lower_vec = deg2rad_with_mask(lower_vec,angle_mask);
    //LOWER DOT LIMIT
    VectorXd lower_dot_vec = get_eigen_vectorxd_from_json_vector(parsed_json["lower_q_dot_limit"].array_items());
    if(angle_mode_degree)
        lower_dot_vec = deg2rad_with_mask(lower_dot_vec,angle_mask);
    //UPPER LIMIT
    VectorXd upper_vec = get_eigen_vectorxd_from_json_vector(parsed_json["upper_q_limit"].array_items());
    if(angle_mode_degree)
        upper_vec = deg2rad_with_mask(upper_vec,angle_mask);
    //UPPER DOT LIMIT
    VectorXd upper_dot_vec = get_eigen_vectorxd_from_json_vector(parsed_json["upper_q_dot_limit"].array_items());
    if(angle_mode_degree)
        upper_dot_vec = deg2rad_with_mask(upper_dot_vec,angle_mask);

    for(const auto& vector : {lower_vec, lower_dot_vec, upper_vec, upper_dot_vec})
    {
        if(serial_manipulator->get_dim_configuration_space()!=vector.size())
        {
            throw std::runtime_error("Incompatible vector sizes in json file for a DQ_SerialManipulator.");
        }
    }

    //End effector
    DQ effector(get_unit_dq_from_json_vector(parsed_json["effector"].array_items()));

    serial_manipulator->set_lower_q_limit(lower_vec);
    serial_manipulator->set_lower_q_dot_limit(lower_dot_vec);
    serial_manipulator->set_upper_q_limit(upper_vec);
    serial_manipulator->set_upper_q_dot_limit(upper_dot_vec);
    serial_manipulator->set_effector(effector);
    
    //DQ_Kinematics commons
    initialize_kinematics_commons(static_cast<DQ_Kinematics*>(serial_manipulator),
                                          parsed_json);
}

DQ_SerialManipulatorDH DQ_JsonReader::_get_serial_manipulator_dh_from_json(const std::string &file)
{
    json11::Json parsed_json = parse_json(file);

    //Type
    std::string type = parsed_json["robot_type"].string_value();
    if(type != "DQ_SerialManipulatorDH")
        throw std::runtime_error("_get_serial_manipulator_dh_from_json not compatible with " + type);

    //ANGLE MODE
    std::string angle_mode_str = parsed_json["angle_mode"].string_value();
    bool angle_mode_degree;
    if(angle_mode_str == "degree")
        angle_mode_degree = true;
    else if(angle_mode_str == "rad")
        angle_mode_degree = false;
    else
        throw std::runtime_error("Unable to decode angle_mode");

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
    VectorXi types = get_eigen_vectorxi_from_json_vector(parsed_json["types"].array_items());

    for(const auto& vector : {theta_vec, d_vec, a_vec, alpha_vec})
    {
        if(types.size()!=vector.size())
        {
            throw std::runtime_error("Incompatible vector sizes in json file for a DQ_SerialManipulatorDH.");
        }
    }

    MatrixXd dh_matrix(5,theta_vec.size());
    dh_matrix << theta_vec.transpose(),
            d_vec.transpose(),
            a_vec.transpose(),
            alpha_vec.transpose(),
            types.cast<double>().transpose();

    DQ_SerialManipulatorDH serial_manipulator_dh(dh_matrix);
    initialize_serial_manipulator_commons(static_cast<DQ_SerialManipulator*>(&serial_manipulator_dh),
                                          parsed_json,
                                          angle_mode_degree,
                                          types);

    return serial_manipulator_dh;
}

DQ_SerialManipulatorDenso DQ_JsonReader::_get_serial_manipulator_denso_from_json(const std::string &file)
{
    json11::Json parsed_json = parse_json(file);

    //Type
    std::string type = parsed_json["robot_type"].string_value();
    if(type != "DQ_SerialManipulatorDenso")
        throw std::runtime_error("_get_serial_manipulator_denso_from_json not compatible with " + type);

    //ANGLE MODE
    std::string angle_mode_str = parsed_json["angle_mode"].string_value();
    bool angle_mode_degree;
    if(angle_mode_str == "degree")
        angle_mode_degree = true;
    else if(angle_mode_str == "rad")
        angle_mode_degree = false;
    else
        throw std::runtime_error("Unable to decode angle_mode");

    //A
    VectorXd a_vec = get_eigen_vectorxd_from_json_vector(parsed_json["a"].array_items());
    //A
    VectorXd b_vec = get_eigen_vectorxd_from_json_vector(parsed_json["b"].array_items());
    //D
    VectorXd d_vec = get_eigen_vectorxd_from_json_vector(parsed_json["d"].array_items());
    //ALPHA
    VectorXd alpha_vec = get_eigen_vectorxd_from_json_vector(parsed_json["alpha"].array_items());
    if(angle_mode_degree)
        alpha_vec = deg2rad(alpha_vec);
    //BETA
    VectorXd beta_vec = get_eigen_vectorxd_from_json_vector(parsed_json["beta"].array_items());
    if(angle_mode_degree)
        beta_vec = deg2rad(beta_vec);
    //GAMMA
    VectorXd gamma_vec = get_eigen_vectorxd_from_json_vector(parsed_json["gamma"].array_items());
    if(angle_mode_degree)
        gamma_vec = deg2rad(gamma_vec);

    for(const auto& vector : {b_vec, d_vec, alpha_vec, beta_vec, gamma_vec})
    {
        if(a_vec.size()!=vector.size())
        {
            throw std::runtime_error("Incompatible vector sizes in json file for a DQ_SerialManipulatorDenso.");
        }
    }

    MatrixXd denso_matrix(6,a_vec.size());
    denso_matrix << a_vec.transpose(),
            b_vec.transpose(),
            d_vec.transpose(),
            alpha_vec.transpose(),
            beta_vec.transpose(),
            gamma_vec.transpose();

    DQ_SerialManipulatorDenso serial_manipulator_denso(denso_matrix);
    initialize_serial_manipulator_commons(static_cast<DQ_SerialManipulator*>(&serial_manipulator_denso),
                                          parsed_json,
                                          angle_mode_degree,
                                          VectorXi::Zero(a_vec.size()));

    return serial_manipulator_denso;
}

template<typename T>
T DQ_JsonReader::get_from_json(const std::string &)
{
    throw std::runtime_error("get_from_json not defined for chosen type.\n"
                             "It is currently defined for:\n"
                             "DQ_SerialKinematicsDH\n"
                             "DQ_SerialKinematicsDenso\n");
}

template <>
DQ_SerialManipulatorDH DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(const std::string& file)
{
    return _get_serial_manipulator_dh_from_json(file);
}

template<>
DQ_SerialManipulatorDenso DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>(const std::string& file)
{
    return _get_serial_manipulator_denso_from_json(file);
}

}
