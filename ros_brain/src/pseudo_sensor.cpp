//
// Created by raimund on 4/2/20.
//

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdlib.h>
#include <vector>
#include <random>

int main(int argc, char **argv)
{
    int resX = 6;
    int resY = 6;
    ros::init(argc, argv, "pseudo_sensor");
    ros::NodeHandle n;
    ros::Publisher pseudo_sensor_pub = n.advertise<std_msgs::Float64MultiArray>("sensor_data", 1000);
    ros::Rate loop_rate(10);
    std::vector<double> pseudo_data;
    std::random_device seed;
    std::mt19937 rand_num(seed());
    std::uniform_real_distribution<> map_to_range(0.0, 1.0);
    int count = 0;
    std_msgs::Float64MultiArray sensorout_msg;
    sensorout_msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); //x
    sensorout_msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); //y
    sensorout_msg.layout.dim[0].size = resX;
    sensorout_msg.layout.dim[0].label = "x";
    sensorout_msg.layout.dim[1].size = resY;
    sensorout_msg.layout.dim[1].label = "y";

    while (ros::ok())
    {
        pseudo_data.clear();
        for(int i=0; i<resX*resY; i++){
            double new_number = map_to_range(rand_num);
            pseudo_data.push_back(new_number);
        }
        // see https://answers.ros.org/question/226726/push-vector-into-multiarray-message-and-publish-it/
        sensorout_msg.data.clear();
        sensorout_msg.data.insert(sensorout_msg.data.end(), pseudo_data.begin(), pseudo_data.end());
        pseudo_sensor_pub.publish(sensorout_msg);
        ROS_INFO("sensor output published. [%f] [%f]", sensorout_msg.data[0], sensorout_msg.data[1]);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}