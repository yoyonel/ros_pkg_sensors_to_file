#ifndef ROS_SENSOR_TO_FILE_H
#define ROS_SENSOR_TO_FILE_H

#include <string>
#include <fstream>

//#include "ros_boost_serialize.h"
#include "ros_boost_serialize_sensors.h"
#include "ros_macros.h"

#include <ros/node_handle.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//
#include <boost/archive/xml_oarchive.hpp>


using namespace ros_boost_serialize::sensors;

/**
 * @brief The SensorsToFile class
 */
class SensorsToFile {
public:

    /**
     * @brief SensorsToFile
     * @param _priv_nh
     * @param _size_buffer
     */
    SensorsToFile (ros::NodeHandle _priv_nh = ros::NodeHandle("~"), const uint32_t& _size_buffer=50 );

    /**
     * @brief get_extension
     * @return
     * Si XML export => extension du fichier = "xml"
     * Sinon si export binaire => extension = "dat"
     * Sinon => export text => extension = "txt"
     */
    inline std::string get_extension() const { return xml_ ? ".xml" : binary_ ? ".dat" : ".txt"; }

    std::string build_filename() const;

    template<typename T, class Archive> static void dump(const std::vector<T> & _vector, Archive & _ar);

    bool dump(const std::string & _filename);
    //
    inline void dump(boost::archive::xml_oarchive & _ar) const { _ar & BOOST_SERIALIZATION_NVP(v_sensor_msgs_); }
    inline void dump(boost::archive::binary_oarchive & _ar) const { dump<_sensor_msgs_serialize_type, boost::archive::binary_oarchive>(v_sensor_msgs_, _ar); }
    inline void dump(boost::archive::text_oarchive & _ar) const { dump<_sensor_msgs_serialize_type, boost::archive::text_oarchive>(v_sensor_msgs_, _ar); }

    void sensors_cb(const sensor_msgs::ImuConstPtr& _msg);

    std::string sensors_topic_;

    ros::Subscriber sub_;

protected:
    ros::NodeHandle nh_;

private:
    std::string prefix_;

    bool binary_;
    bool show_msg_;
    bool xml_;

    std::vector<_sensor_msgs_serialize_type> v_sensor_msgs_;
};

#endif // ROS_SENSOR_TO_FILE_H
