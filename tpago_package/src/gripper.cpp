#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial rs485_port;

void sendRS485Command(const std_msgs::String::ConstPtr& msg) {
    std::string command = msg->data;
    rs485_port.write(command);
    ROS_INFO_STREAM("Sent command: " << command);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rs485_control_node");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyUSB0";
    int baudrate = 9600;

    // 设置串口连接
    rs485_port.setPort(port);
    rs485_port.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    rs485_port.setTimeout(to);

    try {
        rs485_port.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(rs485_port.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Subscriber sub = nh.subscribe("/gripper_command", 10, sendRS485Command);
    ros::spin();

    rs485_port.close();
    return 0;
}
