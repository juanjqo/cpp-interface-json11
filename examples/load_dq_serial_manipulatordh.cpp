#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;

int main(void)
{
    DQ_SerialManipulatorDH robot = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>("../cpp-interface-json11/examples/dq_serial_manipulatordh.json");
    return 0;
}
