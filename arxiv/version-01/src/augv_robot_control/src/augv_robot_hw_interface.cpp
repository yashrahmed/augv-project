#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

using namespace std;

// Taken from https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
vector<string> split(string s, string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
    {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
}

class SimpleAUGVHardwareInterface : public hardware_interface::RobotHW
{
public:
    SimpleAUGVHardwareInterface()
    {
        ROS_INFO("Registering joints.....");
        hardware_interface::JointStateHandle left_whl_state_handle("left_wheel_joint", &pos[0], &vel[0], &effort[0]);
        joint_state_interface.registerHandle(left_whl_state_handle);

        hardware_interface::JointStateHandle right_whl_state_handle("right_wheel_joint", &pos[1], &vel[1], &effort[1]);
        joint_state_interface.registerHandle(right_whl_state_handle);

        registerInterface(&joint_state_interface);

        hardware_interface::JointHandle left_whl_joint_handle(joint_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
        vel_joint_interface.registerHandle(left_whl_joint_handle);

        hardware_interface::JointHandle right_whl_joint_handle(joint_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
        vel_joint_interface.registerHandle(right_whl_joint_handle);
        registerInterface(&vel_joint_interface);
    }

    void publishVelocityControlMessage()
    {
        if (ros::ok())
        {
            std_msgs::String msg;
            stringstream ss;
            ss << setprecision(5) << cmd[0] << "," << setprecision(5) << cmd[1];
            msg.data = ss.str();
            velocityCmdPublisher->publish(msg);
        }
    }

    void readRobotStateCallback(const std_msgs::String::ConstPtr &msg)
    {
        string message = msg->data.c_str();
        vector<string> stateValues = split(message, ",");
        // -- format is left_vel, right_vel, left_pos, right_pos
        // @ Todo - joint position also required for odometry
        vel[0] = stod(stateValues.at(0).c_str());
        vel[1] = stod(stateValues.at(1).c_str());
        pos[0] = stod(stateValues.at(2).c_str());
        pos[1] = stod(stateValues.at(3).c_str());
        //ROS_INFO("Motor State := [lv=%s rv=%s lp=%s rp=%s]", stateValues.at(0).c_str(), stateValues.at(1).c_str(), stateValues.at(2).c_str(), stateValues.at(3).c_str());
    }

    void setCmdPublisher(ros::Publisher *publisher)
    {
        velocityCmdPublisher = publisher;
    }

    void setManager(controller_manager::ControllerManager *managerObj)
    {
        manager = managerObj;
    }

    virtual void update(const ros::TimerEvent &e)
    {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        manager->update(ros::Time::now(), elapsed_time_);
        write(ros::Time::now(), elapsed_time_);
    }

    /*
     read() and write overrides for the RobotHW parent class methods used by ros_control.
     @Note - read is not necessary as the pos/vel/effort values are updates by a topic subscriber..
    */

    virtual void write(const ros::Time &time, const ros::Duration &period)
    {
        publishVelocityControlMessage();
    }

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface vel_joint_interface;

    double cmd[2] = {0.0, 0.0};
    double pos[2] = {0.0, 0.0};
    double vel[2] = {0.0, 0.0};
    double effort[2] = {0.0, 0.0};

    ros::Duration control_period_;
    ros::Duration elapsed_time_;
    ros::Publisher *velocityCmdPublisher;
    controller_manager::ControllerManager *manager;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "AUGV_hardware_interface");
    ros::NodeHandle nh;
    ros::Duration period(0);
    ros::Rate loop_rate(10);

    string inputTopicName = "/vel_in", outputTopicName = "/vel_out";

    ros::param::get(ros::this_node::getName() + "/input_topic", inputTopicName);
    ros::param::get(ros::this_node::getName() + "/output_topic", outputTopicName);

    SimpleAUGVHardwareInterface robotHwInterface;

    ros::Subscriber sub = nh.subscribe(inputTopicName, 1000, &SimpleAUGVHardwareInterface::readRobotStateCallback, &robotHwInterface);
    ros::Publisher velCmdPublisher = nh.advertise<std_msgs::String>(outputTopicName, 1000);
    controller_manager::ControllerManager cm(&robotHwInterface, nh);

    robotHwInterface.setManager(&cm);
    robotHwInterface.setCmdPublisher(&velCmdPublisher);

    ros::Duration updateFreq = ros::Duration(0.1); //every 0.05 seconds....
    ros::Timer non_realtime_loop = nh.createTimer(updateFreq, &SimpleAUGVHardwareInterface::update, &robotHwInterface);

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    // or may time out if waiting for the main loop to respond.
    ros::AsyncSpinner spinner(2); // use 2 threads..
    spinner.start();

    ros::waitForShutdown();
}
