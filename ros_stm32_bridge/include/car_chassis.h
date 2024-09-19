#pragma once

#include <string>
#include <serial/serial.h>
#include "geometry_msgs/PointStamped.h"

//Direction of motion
typedef enum
{
    DIR_STOP = 0,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_FORWARD_LEFT,
    DIR_FORWARD_RIGHT,
    DIR_BACKWARD_LEFT,
    DIR_BACKWARD_RIGHT,
    DIR_TURNING_LEFT,
    DIR_TURNING_RIGHT,
}direction_car;

//position of wheel
typedef enum
{
    NONE_POSITION = 0,
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
}wheel_position;

class wheel
{
private:
    wheel_position position=NONE_POSITION;
    uint8_t           bus_id=0;
    /* data */
public:
    wheel(wheel_position position, uint8_t bus_id);
    wheel(/* args */);
    ~wheel();

    int set_bus_id(uint8_t bus_id);
};

class robotic_arm
{
private:
    /* data */
    uint8_t    bus_id=0;
    uint8_t    position=0;
    geometry_msgs::PointStamped  point;
    //positon and pose
public:
    robotic_arm(uint8_t id, uint8_t pos, geometry_msgs::PointStamped point);
    robotic_arm(/* args */);
    ~robotic_arm();

    int set_bus_id(uint8_t id);
};

class car_chassis
{
private:
    serial::Serial* pSerial_port;
    direction_car   move_direction = DIR_STOP;
    uint32_t        move_time = 0;  //ms
    float           move_speed = 0.0; //
    geometry_msgs::PointStamped point; //TODO
    uint8_t         fl_id; // front left wheel bus id
    uint8_t         fr_id; // front right wheel bus id
    uint8_t         bl_id; // back left wheel bus id
    uint8_t         br_id; // back right wheel bus id
public:
    car_chassis(serial::Serial *port, geometry_msgs::PointStamped point);
    car_chassis();
    ~car_chassis();

    int set_car_motion(direction_car dir, uint32_t time, float speed);
    int set_serial_port(serial::Serial &port);
    int set_inital_point(geometry_msgs::PointStamped point);
    int assemb_command(std::string &command);
    int send_command(std::string &command);
    int get_stm32_serial_data(std::string &data);
    int set_arm_destnation(geometry_msgs::PointStamped point); //TODO
    int set_wheel_id(uint8_t fl_id, uint8_t fr_id, uint8_t bl_id, uint8_t br_id);
    int get_wheel_id(uint8_t &fl_id, uint8_t &fr_id, uint8_t &bl_id, uint8_t &br_id);
};