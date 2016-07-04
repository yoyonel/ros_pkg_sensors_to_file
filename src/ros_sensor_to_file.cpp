#include "ros_sensor_to_file.h"


////////////////////////////////////////////////////////////////////////////////
SensorsToFile::SensorsToFile (ros::NodeHandle _priv_nh,const uint32_t& _size_buffer)
{
    v_sensor_msgs_.reserve(_size_buffer);

    // On recupere les parametres ROS
    GET_ROS_PARAM(prefix, prefix_, "SENSORS_", _priv_nh, nh_);
    GET_ROS_PARAM(binary, binary_, true, _priv_nh, nh_);
    GET_ROS_PARAM(xml, xml_, false, _priv_nh, nh_);
    GET_ROS_PARAM(sensors_topic, sensors_topic_, "/android/imu", _priv_nh, nh_);
    GET_ROS_PARAM(show_msg, show_msg_, false, _priv_nh, nh_);

    // On lance le subscriber
    sub_ = nh_.subscribe(sensors_topic_, 1, &SensorsToFile::sensors_cb, this);

    ROS_INFO ("Listening for incoming data on topic %s",
              nh_.resolveName (sensors_topic_).c_str ());
}

std::string SensorsToFile::build_filename() const
{
    // On recupere le timestamp du message
    const ros::Time& stamp_begin = v_sensor_msgs_.front().stamp();
    const ros::Time& stamp_end = v_sensor_msgs_.back().stamp();
    // On construit le nom du fichier image par rapport a ce timestamp
    std::stringstream ss_sensors_filename;
    ss_sensors_filename << prefix_ <<                               \
                           stamp_begin << "_to_" << stamp_end <<    \
                           get_extension();
    return ss_sensors_filename.str();
}

template<typename T, class Archive>
void SensorsToFile::dump(const std::vector<T> & _vector, Archive & _ar)
{
    // dump vector to archive
    for (typename std::vector<T>::const_iterator it = _vector.begin() ; it != _vector.end(); ++it)
        _ar << *it;
}

bool SensorsToFile::dump(const std::string & _filename)
{
    bool retour = true;

    std::ofstream ofs(_filename, std::ios::out | std::ofstream::binary);
    if(ofs.is_open()) {
        // save data to archive
        try {
            if(xml_) {
                boost::archive::xml_oarchive oa(ofs);
                dump(oa);
            }
            else {
                if(binary_) {
                    boost::archive::binary_oarchive oa(ofs);
                    // write vector to archive
                    dump(oa);
                    // archive and stream closed when destructors are called
                }
                else {
                    boost::archive::text_oarchive oa(ofs);
                    // write vector to archive
                    dump(oa);
                    // archive and stream closed when destructors are called
                }
            }
        }
        catch(std::exception &exc){
            ROS_ERROR_STREAM("Exception lors de la serialization ! -> " << exc.what());
            retour = false;
        }

        ROS_INFO_STREAM("Write sensors data into: " << _filename);
    }
    else {
        ROS_ERROR_STREAM("Probleme d'ouverture du stream: " << _filename);
        retour = false;
    }

    return retour;
}

void SensorsToFile::sensors_cb(const sensor_msgs::ImuConstPtr& _msg)
{
    if (show_msg_)
        ROS_INFO_STREAM("Orientation\n" << _msg->orientation);

    // Sauvegarde en memoire du message (les informations qui nous interessent du message)
    _sensor_msgs_serialize_type sensor_msg(_msg);
    v_sensor_msgs_.push_back(sensor_msg);
    //        ROS_INFO_STREAM("sensor_msg : " << sensor_msg);

    // Est ce qu'on a atteint la capacite max. du vecteur ?
    if (v_sensor_msgs_.size() == v_sensor_msgs_.capacity())
    {
        // Si oui, on enregistre (sur fichier) le vecteur
        dump(build_filename());
        // et on le vide
        v_sensor_msgs_.clear();
    }
}
