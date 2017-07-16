/*
 * Author: Natchanan Thongtem
 * Created on: 10/06/2017
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <zeabus_elec_ros_hardware_interface/PowerSwitchCommand.h>
#include <zeabus_elec_ros_hardware_interface/IOCommand.h>

#include <zeabus_elec_ros_power_dist/power_dist.h>
#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid_sw.h>

#define ONE_ATM_AS_PSI 14.6959
#define PSI_PER_DEPTH 0.6859

ros::Publisher odometry_publisher;

ros::Subscriber barometer_subsciber;

ros::ServiceServer set_power_switch_on_service_server;
ros::ServiceServer set_power_switch_off_service_server;
ros::ServiceServer set_solenoid_on_service_server;
ros::ServiceServer set_solenoid_off_service_server;

ros::ServiceClient power_dist_service_client;
ros::ServiceClient solenoid_service_client;

double atm_pressure, depth_offset;

double barometer_value_to_depth(uint16_t barometer_value)
{
    double barometer_voltage, psi, depth;

    barometer_voltage = barometer_value * (5.0 / 1023.0);
    psi = (barometer_voltage - 0.5) * (30.0 / 4.0);

    depth = ((psi - atm_pressure) * PSI_PER_DEPTH) + depth_offset;

    ROS_INFO("pressure sensor analog value : %.4d", barometer_value);
    ROS_INFO("pressure sensor voltage : %.4lf V", barometer_voltage);
    ROS_INFO("pressure : %.4lf psi", psi);
    ROS_INFO("depth : %.4lf meter\n", depth);

    return depth;
}

void send_depth(const zeabus_elec_ros_peripheral_bridge::barometer::ConstPtr& msg)
{
    nav_msgs::Odometry odometry;

    double depth;

    depth = barometer_value_to_depth(msg->pressureValue);
    
    odometry.header.stamp = ros::Time::now();

    odometry.pose.pose.position.z = -depth;

    odometry_publisher.publish(odometry);
}

bool set_power_switch_on(zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Response &res)
{
    zeabus_elec_ros_power_dist::power_dist power_dist_service;

    power_dist_service.request.switchIndex = req.channel;

    if(req.channel >= 6)
    {
        power_dist_service.request.isSwitchHigh = false;
    }
    else
    {
        power_dist_service.request.isSwitchHigh = true;
    }

    res.result = power_dist_service_client.call(power_dist_service);

    return res.result;
}

bool set_power_switch_off(zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Request &req,
                        zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Response &res)
{
    zeabus_elec_ros_power_dist::power_dist power_dist_service;

    power_dist_service.request.switchIndex = req.channel;

    if(req.channel >= 6)
    {
        power_dist_service.request.isSwitchHigh = true;
    }
    else
    {
        power_dist_service.request.isSwitchHigh = false;
    }

    res.result = power_dist_service_client.call(power_dist_service);

    return res.result;
}

bool set_solenoid_on(zeabus_elec_ros_hardware_interface::IOCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoid_service;

    solenoid_service.request.switchIndex = req.channel;
    solenoid_service.request.isSwitchHigh = true;

    res.result = solenoid_service_client.call(solenoid_service);

    return res.result;
}

bool set_solenoid_off(zeabus_elec_ros_hardware_interface::IOCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoid_service;

    solenoid_service.request.switchIndex = req.channel;
    solenoid_service.request.isSwitchHigh = false;

    res.result = solenoid_service_client.call(solenoid_service);

    return res.result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Zeabus_Elec_Hardware_interface");
    ros::NodeHandle nh("/zeabus/elec");

    nh.param<double>("/Zeabus_Elec_Hardware_interface/atm_pressure", atm_pressure, ONE_ATM_AS_PSI);
    nh.param<double>("/Zeabus_Elec_Hardware_interface/depth_offset", depth_offset, 0);

    odometry_publisher = nh.advertise<nav_msgs::Odometry>("/baro/odom", 10);

    barometer_subsciber = nh.subscribe("barometer", 10, send_depth);

    set_power_switch_on_service_server = nh.advertiseService("/power_distribution/switch_on", set_power_switch_on);
    set_power_switch_off_service_server = nh.advertiseService("/power_distribution/switch_off", set_power_switch_off);
    set_solenoid_on_service_server = nh.advertiseService("/io_and_pressure/IO_ON", set_solenoid_on);
    set_solenoid_off_service_server = nh.advertiseService("/io_and_pressure/IO_OFF", set_solenoid_off);

    power_dist_service_client = nh.serviceClient<zeabus_elec_ros_power_dist::power_dist>("power_switch");
    solenoid_service_client = nh.serviceClient<zeabus_elec_ros_peripheral_bridge::solenoid_sw>("solenoid_sw");

    ros::spin();

    return 0;
}
