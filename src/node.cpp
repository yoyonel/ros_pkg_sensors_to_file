// urls:
// - http://www.boost.org/doc/libs/1_61_0/libs/serialization/doc/tutorial.html
#include <ros/ros.h>
#include <node_sensor_to_file.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle priv_nh("~");

    SensorsToFile stof(priv_nh);

    ros::spin();
}
