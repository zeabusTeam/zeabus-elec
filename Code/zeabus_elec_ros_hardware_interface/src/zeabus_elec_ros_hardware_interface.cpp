/*
 * Author: Natchanan Thongtem
 * Created on: 10/06/2017
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <zeabus_elec_ros_hardware_interface/IOCommand.h>

#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid_sw.h>

#define PSI_AT_ATMOSPHERE 14.6959
#define PSI_PER_DEPTH 0.6859

ros::Publisher odometryPublisher;

ros::Subscriber barometerSubscriber;

ros::ServiceServer setSolenoidOnServiceServer;
ros::ServiceServer setSolenoidOffServiceServer;

ros::ServiceClient SolenoidServiceClient;

nav_msgs::Odometry odometry;

void ZeabusElec_BarometerValToDepth(const zeabus_elec_ros_peripheral_bridge::barometer::ConstPtr& msg)
{
    double baromenterVoltage, psi, depth;
    uint16_t barometerVal;

    barometerVal = msg->pressureValue;

    baromenterVoltage = barometerVal * (5.0 / 1023.0);
    psi = (baromenterVoltage - 0.5) * (30.0 / 4.0);

    depth = (psi - PSI_AT_ATMOSPHERE) * PSI_PER_DEPTH;
    
    odometry.header.stamp = ros::Time::now();

    odometry.pose.pose.position.z = -depth;

    odometryPublisher.publish(odometry);

    ROS_INFO("pressure sensor analog value : %.4d\n", msg->pressureValue);
    ROS_INFO("pressure sensor voltage : %.4lf V\n", baromenterVoltage);
    ROS_INFO("pressure : %.4lf psi\n", psi);
    ROS_INFO("depth : %.4lf meter\n", depth);
}

bool ZeabusElec_setSolenoidOn(zeabus_elec_ros_hardware_interface::IOCommand::Request &req, zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoidService;

    solenoidService.request.switchIndex = req.channel;
    solenoidService.request.isSwitchHigh = true;

    res.result = SolenoidServiceClient.call(solenoidService);

    return res.result;
}

bool ZeabusElec_setSolenoidOff(zeabus_elec_ros_hardware_interface::IOCommand::Request &req, zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoidService;

    solenoidService.request.switchIndex = req.channel;
    solenoidService.request.isSwitchHigh = false;

    res.result = SolenoidServiceClient.call(solenoidService);

    return res.result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Zeabus_Elec_Hardware_interface");
    ros::NodeHandle nodeHandle("/zeabus/elec");
    
    odometryPublisher = nodeHandle.advertise<nav_msgs::Odometry>("/baro/odom", 10);

    barometerSubscriber = nodeHandle.subscribe("/barometer", 10, ZeabusElec_BarometerValToDepth);

    setSolenoidOnServiceServer = nodeHandle.advertiseService("/io_and_pressure/IO_ON", ZeabusElec_setSolenoidOn);
    setSolenoidOffServiceServer = nodeHandle.advertiseService("/io_and_pressure/IO_OFF", ZeabusElec_setSolenoidOff);

    SolenoidServiceClient = nodeHandle.serviceClient<zeabus_elec_ros_peripheral_bridge::solenoid_sw>("/solenoid_sw");

    ros::spin();

    return 0;
}
