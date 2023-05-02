#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "NeuralNetwork.h"
#include "Genome.h"

using namespace NEAT;

// where best to declare this?
NeuralNetwork Brain;
std::vector<double> NN_input;
std::vector<double> NN_output;
unsigned int frequency = 50;
bool verbose = true;

void brainCallback(const std_msgs::Float64MultiArray::ConstPtr& brain_in)
{
    //fill brain input vector with data from message and truncate to input layer size
    NN_input.clear();
    NN_input.insert(NN_input.begin(), brain_in->data.begin(), brain_in->data.end());
    NN_input.resize(Brain.NumInputs());
    NN_output.resize(Brain.NumOutputs());

    //Brain.ActivateFast() only called when new message published in subscription. Move to main loop?
    Brain.Input(NN_input);
    NN_input.back() = 1.0; //last input is always a bias node
    Brain.ActivateFast();
    NN_output = Brain.Output();

    if (verbose){
        std::cout << "---" << std::endl;
        for (int i = 0; i < Brain.NumInputs(); i++) {
            std::cout << "in" << i << ": " << NN_input[i] << std::endl;
        }
        for (int i = 0; i < Brain.NumOutputs(); i++) {
            std::cout << "out" << i << ": " << NN_output[i] << std::endl;
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "brain");
    ros::NodeHandle n_private("~");
    //06.04.20 "~" creates a private NodeHandle, needed to pass command line arguments, see:
    //https://robotics.stackexchange.com/questions/19113/command-line-boolean-parameters-to-ros-node
    ros::NodeHandle n;
    ros::Rate loop_rate(frequency);
    std::string genome_param;
    genome_param.clear();
    n_private.getParam("genome", genome_param);

    //first, let's create the NN from the genome file
    const char* Genome_File_cstr = genome_param.c_str();
    Genome myGenome(Genome_File_cstr);
    myGenome.BuildPhenotype(Brain);
    ROS_INFO("Brain generated from file: [%s]", genome_param.c_str());
    ROS_INFO("%i inputs, %i outputs, %i neurons total", Brain.NumInputs(), Brain.NumOutputs(), Brain.NumNeurons());
    ros::Publisher brainout_pub = n.advertise<std_msgs::Float64MultiArray>("brain_out", 1000);
    ros::Subscriber brainin_sub = n.subscribe("sensor_data", 1000, brainCallback);
    std_msgs::Float64MultiArray brainout_msg;
    brainout_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    brainout_msg.layout.dim[0].size = Brain.NumOutputs();
    brainout_msg.layout.dim[0].label = "brain_out_vector";

    while (ros::ok()){
        brainout_msg.data.clear();
        brainout_msg.data.insert(brainout_msg.data.begin(), NN_output.begin(), NN_output.end());
        brainout_pub.publish(brainout_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
