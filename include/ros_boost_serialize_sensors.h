#ifndef ROS_BOOST_SERIALIZE_SENSORS_H
#define ROS_BOOST_SERIALIZE_SENSORS_H

#include "ros_boost_serialize.h"
#include <boost/date_time/posix_time/time_serialize.hpp>

namespace ros_boost_serialize {
namespace sensors {

class stamp_imu_serialize_type
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        boost::serialization::save<Archive>(ar, stamp.time_of_day(), version);
    }

public:
    stamp_imu_serialize_type(const sensor_msgs::ImuConstPtr & msg_) :
        stamp(msg_->header.stamp.toBoost())
    {}

private:
    const boost::posix_time::ptime stamp;
};

class _sensor_msgs_serialize_type {
    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version) {
        // ------------------------
        // Informations temporelles
        // ------------------------
        ar << BOOST_SERIALIZATION_NVP(stamp_imu);

        // ------------------------
        // Informations IMU
        // ------------------------
        ar <<  BOOST_SERIALIZATION_NVP(orientation);
        ar <<  BOOST_SERIALIZATION_NVP(angular_velocity);
        ar <<  BOOST_SERIALIZATION_NVP(linear_velocity);
    }

public:
    // ------------------------
    // CONSTRUCTOR
    // ------------------------
    _sensor_msgs_serialize_type(const sensor_msgs::ImuConstPtr& _msg) :
        msg_(_msg)
      , orientation(_msg), angular_velocity(_msg), linear_velocity(_msg)
      , stamp_imu(_msg)
    {}

    // ------------------------
    // GETTERS
    // ------------------------
    inline const ros::Time&                 stamp() const { return msg_->header.stamp; }
    inline const sensor_msgs::ImuConstPtr&  msg()   const { return msg_; }

    friend std::ostream & operator<<(std::ostream &os, const _sensor_msgs_serialize_type &msg_);

private:
    sensor_msgs::ImuConstPtr msg_;

    typedef vector3_ros_serialize_type<sensor_msgs::Imu> angular_velocity_imu_serialize_type;
    typedef vector3_ros_serialize_type<sensor_msgs::Imu> linear_acceleration_imu_serialize_type;
    typedef vector4_ros_serialize_type<sensor_msgs::Imu> orientation_imu_serialize_type;

    orientation_imu_serialize_type orientation;
    angular_velocity_imu_serialize_type angular_velocity;
    linear_acceleration_imu_serialize_type linear_velocity;
    stamp_imu_serialize_type stamp_imu;
};

}
}

BOOST_CLASS_VERSION(ros_boost_serialize::sensors::stamp_imu_serialize_type, 1);
BOOST_CLASS_VERSION(ros_boost_serialize::sensors::_sensor_msgs_serialize_type, 1);

#endif // ROS_BOOST_SERIALIZE_SENSORS_H
