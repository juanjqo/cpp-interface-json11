#include <dqrobotics/interfaces/DQ_JsonReader.h>

#include <json11.hpp>

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<exception>
#include<vector>

//Usage example
//int main(void)
//{
//    DQ_robotics::DQ_SerialManipulator a = DQ_robotics::DQ_JsonReader::get_serial_manipulator_from_json("/home/murilo/git/dqrobotics/cpp-interface-json11/robot.json");
//    return 1;
//}

namespace DQ_robotics
{

VectorXd get_eigen_vector_from_json_vector(const std::vector<json11::Json>& json_vector)
{
    VectorXd eigen_vector(json_vector.size());
    for(int i=0;i<json_vector.size();i++)
    {
        eigen_vector(i) = json_vector[i].number_value();
    }
    return eigen_vector;
}

double d2r(const double &d)
{
    return ((M_PI*d)/180.0);
}

Eigen::VectorXd d2r(const Eigen::VectorXd& indegrees)
{
    return indegrees*(M_PI/180.0);
}

DQ_SerialManipulator DQ_JsonReader::get_serial_manipulator_from_json(const std::string &file)
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
        throw std::runtime_error("Json parse error");

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
    VectorXd theta_vec = get_eigen_vector_from_json_vector(parsed_json["theta"].array_items());
    if(angle_mode_degree)
        theta_vec = d2r(theta_vec);
    //D
    VectorXd d_vec = get_eigen_vector_from_json_vector(parsed_json["d"].array_items());
    //A
    VectorXd a_vec = get_eigen_vector_from_json_vector(parsed_json["a"].array_items());
    //ALPHA
    VectorXd alpha_vec = get_eigen_vector_from_json_vector(parsed_json["alpha"].array_items());
    if(angle_mode_degree)
        alpha_vec = d2r(alpha_vec);

    MatrixXd dh_matrix(4,theta_vec.size());
    dh_matrix << theta_vec.transpose(),
            d_vec.transpose(),
            a_vec.transpose(),
            alpha_vec.transpose();

    //LOWER LIMIT
    VectorXd lower_vec = get_eigen_vector_from_json_vector(parsed_json["lower_q_limit"].array_items());
    if(angle_mode_degree)
        lower_vec = d2r(lower_vec);
    //UPPER LIMIT
    VectorXd upper_vec = get_eigen_vector_from_json_vector(parsed_json["upper_q_limit"].array_items());
    if(angle_mode_degree)
        upper_vec = d2r(upper_vec);

    DQ_SerialManipulator serial_manipulator(dh_matrix, convention);
    serial_manipulator.set_lower_q_limit(lower_vec);
    serial_manipulator.set_upper_q_limit(upper_vec);

    return serial_manipulator;
}

}
