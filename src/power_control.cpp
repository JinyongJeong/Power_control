#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>

using namespace std;

serial::Serial ser;
bool process_active_flag = false;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    if(process_active_flag == true){
        ROS_INFO_STREAM("Writing to serial port" << msg->data);
        ser.write(msg->data);
    }
}

bool status_check_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = process_active_flag;
}

bool process_run_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{    
    process_active_flag = req.data;
    res.success = process_active_flag;
    if(process_active_flag == true){
        cout <<"Receive process active" << endl;
    }else{
        cout <<"Receive process inactive" << endl;
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "power_control_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("/power_control_write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/power_control_read", 1000);

    ros::ServiceServer status_check_srv = nh.advertiseService("/power_control_status_check", status_check_callback);
    ros::ServiceServer process_run_srv = nh.advertiseService("/power_control_run", process_run_callback);

    std::string device;
    int baudrate;

    nh.param("device", device, std::string("/dev/ttyUSB-power"));
    nh.param("baudrate", baudrate, 115200);

    try
    {
        ser.setPort(device);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        if(ser.available() && process_active_flag == true){        
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            //ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}

