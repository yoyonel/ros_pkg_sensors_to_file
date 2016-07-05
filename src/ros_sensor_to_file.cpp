#include "ros_sensor_to_file.h"


////////////////////////////////////////////////////////////////////////////////
SensorsToFile::SensorsToFile (ros::NodeHandle _priv_nh, const uint32_t& _size_buffer) :
    SubToFile(_size_buffer)
{
    // On recupere les parametres ROS
    GET_ROS_PARAM(prefix, prefix_, "SENSORS_", _priv_nh, nh_);

    // Lance le subscriber ros sur le topic
    run();
}

std::string SensorsToFile::build_filename() const
{
    // On recupere le timestamp du message
    const ros::Time& stamp_begin = v_msgs_.front().stamp();
    const ros::Time& stamp_end = v_msgs_.back().stamp();
    // On construit le nom du fichier image par rapport a ce timestamp
    std::stringstream ss_sensors_filename;
    ss_sensors_filename << prefix_ <<                               \
                           stamp_begin << "_to_" << stamp_end <<    \
                           get_extension();
    return ss_sensors_filename.str();
}
