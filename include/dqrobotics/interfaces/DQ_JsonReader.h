#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class DQ_JsonReader
{
public:
    static DQ_SerialManipulator get_serial_manipulator_from_json(const std::string& file);
};

}
