/*
 * @Author: pandy 14674879+pandy2@user.noreply.gitee.com
 * @Date: 2024-07-18 09:38:14
 * @LastEditors: pandy 14674879+pandy2@user.noreply.gitee.com
 * @LastEditTime: 2024-07-20 12:39:27
 * @FilePath: /robot-zero/src/ros_stm32_bridge/src/cmd_pub.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include "ros_stm32_bridge/motion.h"

const static std::string cmd_list[11]=
{
    "stop",
    "forward", 
    "backward",
    "left",
    "right",
    "forward_left",
    "forward_right",
    "backward_left", 
    "backward_right", 
    "truning_left",
    "truning_right",
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "car_pub_node");
    ros::NodeHandle n;

    //subcribe Message
    // ros::Publisher command_pub = n.advertise<std_msgs::String>("cmd_car", 1000);
    ros::Publisher command_pub = n.advertise<ros_stm32_bridge::motion>("car_motion", 1000);

    ros_stm32_bridge::motion motion_msgs;
    uint8_t i = 0;

    ros::Rate r(1);

    motion_msgs.time = 500;
    motion_msgs.revolution = 10;

    while(ros::ok())
    {
        motion_msgs.direction = cmd_list[i];
        if(++i >= 11)
        {
            i = 0;
        }

        ROS_INFO("Send message is:%s", motion_msgs.direction.c_str());
        command_pub.publish(motion_msgs);
        r.sleep();
        ros::spinOnce();
    }
}