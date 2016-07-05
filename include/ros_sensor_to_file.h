#ifndef ROS_SENSOR_TO_FILE_H
#define ROS_SENSOR_TO_FILE_H

#include <string>
#include <fstream>

#include "ros_boost_serialize_sensors.h"

#include <subscriber_to_file/subscriber_to_file.h>

using namespace ros_boost_serialize::sensors;

/**
 * @brief The SensorsToFile class
 */
class SensorsToFile : public SubToFile< ::sensor_msgs::Imu, _sensor_msgs_serialize_type >
{
public:

    /**
     * @brief SensorsToFile
     * @param _priv_nh
     * @param _size_buffer
     */
    SensorsToFile (ros::NodeHandle _priv_nh = ros::NodeHandle("~"), const uint32_t& _size_buffer=50 );

    /**
     * @brief build_filename
     * @return
     */
    std::string build_filename() const override;

protected:
    /**
     * @brief get_extension
     * @return
     * Si XML export => extension du fichier = "xml"
     * Sinon si export binaire => extension = "dat"
     * Sinon => export text => extension = "txt"
     */
    inline std::string get_extension() const { return xml_ ? ".xml" : binary_ ? ".dat" : ".txt"; }

private:
    std::string prefix_;
};

#endif
