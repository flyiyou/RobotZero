/*
 * @Author: pandy 14674879+pandy2@user.noreply.gitee.com
 * @Date: 2024-07-18 09:38:14
 * @LastEditors: pandy 14674879+pandy2@user.noreply.gitee.com
 * @LastEditTime: 2024-07-20 12:28:57
 * @FilePath: /robot-zero/src/ros_stm32_bridge/src/cmd_sub.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <string>
#include "car_chassis.h"
#include "ros_stm32_bridge/motion.h"
#include "stm32_cmd.h"

serial::Serial ros_ser;
geometry_msgs::PointStamped car_point;
car_chassis car_node;
std::string serial_tx_data;
std::stringstream stream;
std::string cmd_time_str;
std::string fl_cmd, fr_cmd, bl_cmd, br_cmd;
std::string fl_id_str, fr_id_str, bl_id_str, br_id_str;

static uint32_t fl_rev, fr_rev, bl_rev, br_rev;
static uint8_t fl_id, fr_id, bl_id, br_id;

const static std::map<std::string , motor_rotation_st>  moter_dir_map
{
    {"stop", {MOTOR_STOP, MOTOR_STOP, MOTOR_STOP, MOTOR_STOP}},
    {"forward", {MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE}},
    {"backward", {MOTOR_COUNTER_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_CLOCKWISE}},
    {"left", {MOTOR_COUNTER_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_CLOCKWISE}},
    {"right", {MOTOR_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE}},
    {"forward_left", {MOTOR_STOP, MOTOR_COUNTER_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_STOP}},
    {"forward_right", {MOTOR_CLOCKWISE, MOTOR_STOP, MOTOR_STOP, MOTOR_COUNTER_CLOCKWISE}},
    {"backward_left", {MOTOR_COUNTER_CLOCKWISE, MOTOR_STOP, MOTOR_STOP, MOTOR_CLOCKWISE}},
    {"backward_right", {MOTOR_STOP, MOTOR_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_STOP}},
    {"truning_left", {MOTOR_COUNTER_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE, MOTOR_COUNTER_CLOCKWISE}},
    {"truning_right", {MOTOR_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_CLOCKWISE, MOTOR_CLOCKWISE}},
};

static int parse_msgs_to_cmd(const ros_stm32_bridge::motion &msgs, std::string &cmd)
{
    uint32_t motion_time = msgs.time;
    std::string tmp_str;

    if(motion_time > 9999)
    {
       motion_time  = 9999;
    }

    cmd_time_str = std::to_string(10000 + motion_time).substr(1,4);
    fl_id_str = std::to_string(1000 + fl_id).substr(1,3);
    fr_id_str = std::to_string(1000 + fr_id).substr(1,3);
    bl_id_str = std::to_string(1000 + bl_id).substr(1,3);
    br_id_str = std::to_string(1000 + br_id).substr(1,3);

    if(msgs.direction == "stop")
    {
        cmd = "$DST!";// TODO only stop wheel

    }
    else if(moter_dir_map.find(msgs.direction) != moter_dir_map.end()) // check cmd in map
    {
        
        fl_rev = 1500 + msgs.revolution * ((moter_dir_map.find(msgs.direction)->second).fl_rotation);
        fr_rev = 1500 + msgs.revolution * ((moter_dir_map.find(msgs.direction)->second).fr_rotation);
        bl_rev = 1500 + msgs.revolution * ((moter_dir_map.find(msgs.direction)->second).bl_rotation);
        br_rev = 1500 + msgs.revolution * ((moter_dir_map.find(msgs.direction)->second).br_rotation);
        cmd = "{#" + fl_id_str + "P" + std::to_string(fl_rev) + "B" + cmd_time_str + "!" \
              "#" + fr_id_str + "P" + std::to_string(fr_rev) + "B" + cmd_time_str + "!" \
              "#" + bl_id_str + "P" + std::to_string(bl_rev) + "B" + cmd_time_str + "!" \
              "#" + br_id_str + "P" + std::to_string(br_rev) + "B" + cmd_time_str + "!}";
    }    

    return 0;
}


void callback(const ros_stm32_bridge::motion &msgs)
{
    ROS_INFO("Receive message is:\n[direction] is:[%s]\n[time] is %dms\n[revolution] is %d\n", 
            msgs.direction.c_str(), msgs.time, msgs.revolution*50);
    // ros_ser.write(msgs_p->data.c_str());
    if(msgs.direction == "Dir_Forward")
    {
        car_node.set_car_motion(DIR_FORWARD, 1000, 1);
    }
    else if (msgs.direction == "Dir_Backward")
    {
        car_node.set_car_motion(DIR_BACKWARD, 1000, 1);
    }

    // car_node.assemb_command(serial_tx_data);

    parse_msgs_to_cmd(msgs, serial_tx_data);

    
    ROS_INFO("Send message is:%s\n", serial_tx_data.c_str());
    car_node.send_command(serial_tx_data);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "car_sub_node");
    ros::NodeHandle n;

    //subcribe Message
    ros::Subscriber command_sub = n.subscribe("/car_motion", 1000, callback);

    try
    {
        ros_ser.setPort("/dev/ttyTHS0");
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch(const std::exception& e)
    {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
    }

    if(ros_ser.isOpen())
    {
        ROS_INFO_STREAM(" Serial Port \"ttyTHS0\" open sucess!");
    }
    else
    {
        return -1;
    }
    car_node.set_wheel_id(6 ,7, 8, 9);
    car_node.set_serial_port(ros_ser);
    car_node.get_wheel_id(fl_id, fr_id, bl_id, br_id);

    ros::spin();
}