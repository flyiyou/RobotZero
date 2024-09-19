#include <string>
#include "car_chassis.h"

wheel::wheel(/* args */)
{
    
}
wheel::~wheel()
{

}
wheel:: wheel(wheel_position pos, uint8_t id)
{
    this->position = position;
    this->bus_id  = id;
}

robotic_arm::robotic_arm(/* args */)
{
}

robotic_arm::~robotic_arm()
{
}

int robotic_arm::set_bus_id(uint8_t id)
{
    this->bus_id = id;

    return 0;
}

car_chassis::car_chassis()
{
    this->pSerial_port = nullptr;
    this->move_direction = DIR_STOP;
    this->move_speed = 0;
    this->move_time = 0;
}

car_chassis::car_chassis(serial::Serial *port, geometry_msgs::PointStamped point)
{
    this->pSerial_port = port;
    this->point = point;
}

car_chassis::~car_chassis()
{

}

int car_chassis::set_serial_port(serial::Serial &port)
{
    this->pSerial_port = &port;
    return 0;
}
int car_chassis::set_inital_point(geometry_msgs::PointStamped point)
{
    return 0;
}

int car_chassis::set_car_motion(direction_car dir, uint32_t time, float speed)
{
    this->move_direction = dir;
    this->move_time = time;
    this->move_speed = speed;
    
    return 0;
}

int car_chassis::assemb_command(std::string &command)
{
    std::string fl_cmd, fr_cmd, bl_cmd, br_cmd;

    switch (this->move_direction)
    {
    case DIR_FORWARD:
        command = "{#006P1510B0500!#007P1490B0500!#008P1510B0500!#009P1490B0500!}";
        break;
    case DIR_BACKWARD:
        command = "{#006P1490B0500!#007P1510B0500!#008P1490B0500!#009P1510B0500!}";
        break;
    case DIR_LEFT:
        /* code */
        break;
    case DIR_RIGHT:
        /* code */
        break;
    case DIR_FORWARD_LEFT:
        /* code */
        break;
    case DIR_FORWARD_RIGHT:
        /* code */
        break;
    case DIR_BACKWARD_LEFT:
        /* code */
        break;
    case DIR_BACKWARD_RIGHT:
        /* code */
        break;
    case DIR_TURNING_LEFT:
        /* code */
        break;
    case DIR_TURNING_RIGHT:
        /* code */
        break;
    
    default:
        command = "$DST!";
        break;
    }

    return 0;
}

int car_chassis::send_command(std::string &command)//TODO
{
    if(!this->pSerial_port->isOpen())
    {
        return -1;
    }

    this->pSerial_port->write(command.c_str());

    return 0;
}

int car_chassis::get_stm32_serial_data(std::string &data)//TODO
{
    return 0;
}

int car_chassis::set_arm_destnation(geometry_msgs::PointStamped point) //TODO
{
    return 0;
}

int car_chassis::set_wheel_id(uint8_t fl_id, uint8_t fr_id, uint8_t bl_id, uint8_t br_id)
{
    this->fl_id = fl_id;
    this->fr_id = fr_id;
    this->bl_id = bl_id;
    this->br_id = br_id;
    return 0;
}

int car_chassis::get_wheel_id(uint8_t &fl_id, uint8_t &fr_id, uint8_t &bl_id, uint8_t &br_id)
{
    fl_id = this->fl_id;
    fr_id = this->fr_id;
    bl_id = this->bl_id;
    br_id = this->br_id;
    return 0;
}