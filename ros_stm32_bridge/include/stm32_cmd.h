/*
 * @Author: pandy 14674879+pandy2@user.noreply.gitee.com
 * @Date: 2024-07-19 21:54:02
 * @LastEditors: pandy 14674879+pandy2@user.noreply.gitee.com
 * @LastEditTime: 2024-07-19 23:16:45
 * @FilePath: /robot-zero/src/ros_stm32_bridge/include/stm32_cmd.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <map>
#include <string>

#define MOTOR_STOP              (0)
#define MOTOR_CLOCKWISE         (1)
#define MOTOR_COUNTER_CLOCKWISE (-1)

typedef struct
{
    int  fl_rotation;
    int  fr_rotation;
    int  bl_rotation;
    int  br_rotation;
}motor_rotation_st;
