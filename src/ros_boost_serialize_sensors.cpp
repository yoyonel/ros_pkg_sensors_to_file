#include "ros_boost_serialize_sensors.h"

using namespace ros_boost_serialize::sensors;

std::ostream & operator<<(std::ostream &os, const _sensor_msgs_serialize_type &msg_)
{
    // ostream of ROS msg
    return os << *(msg_.msg());
}
