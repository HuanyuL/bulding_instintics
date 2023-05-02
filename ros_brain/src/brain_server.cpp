//
// Created by raimund on 4/13/20.
//
#include "ros/ros.h"
#include "NeuralNetwork.h"
#include "Genome.h"
#include "ros_brain/TriggerBrain.h" // see ~/ros_workspace/devel/include

using namespace NEAT;
NeuralNetwork Brain;
std::vector<double> NN_input;
std::vector<double> NN_output;
unsigned int frequency = 50;

bool activate_brain(ros_brain::TriggerBrain::Request &req, ros_brain::TriggerBrain::Response &res) {

    //fill brain input vector with data from service request and truncate to input layer size
    NN_input.clear();
    NN_input.insert(NN_input.begin(),req.in_vec.begin(), req.in_vec.end());
    NN_input.resize(Brain.NumInputs());
    NN_output.resize(Brain.NumOutputs());
    Brain.Input(NN_input);
    NN_input.back() = 1.0; //last input is always a bias node

    //get output and stuff it in the response (see TriggerBrain.srv for definition)
    Brain.ActivateFast();
    NN_output = Brain.Output();
    res.out_vec = NN_output;
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "brain_server");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    // define name of service and associated function (type of service is ros_brain/TriggerBrain)
    ros::ServiceServer service = n.advertiseService("trigger_brain", activate_brain);

    //first, create the NN from the genome file:
    std::string genome_param;
    genome_param.clear();
    n_private.getParam("genome", genome_param);
    const char* Genome_File_cstr = genome_param.c_str();
    Genome myGenome(Genome_File_cstr);
    myGenome.BuildPhenotype(Brain);
    ROS_INFO("Brain generated from file: [%s]", genome_param.c_str());
    ROS_INFO("%i inputs, %i outputs, %i neurons total", Brain.NumInputs(), Brain.NumOutputs(), Brain.NumNeurons());
    ROS_INFO("ready!");
    ros::spin();
    return 0;
}