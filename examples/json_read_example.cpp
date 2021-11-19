#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;

int main(void)
{
    DQ_SerialManipulatorDH robot_dh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("../cpp-interface-json11/examples/dq_serial_manipulator_dh.json");
    DQ_SerialManipulatorDenso robot_denso = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>("../cpp-interface-json11/examples/dq_serial_manipulator_denso.json");
    return 0;
}
