#ifndef _leg_message
#define _leg_message

#include <stdint.h>

// SPI 从设备（腿控板）上传给主机的 30 个 16-bit 字的状态帧，约 60 字节。
// Google 风格要求：类型说明紧跟在类型前，必要时在字段上补充含义。
struct spi_data_t {
    float q_abad[2];      // 髋外展关节角度，左右腿各 1 个
    float q_hip[2];       // 髋关节角度
    float q_knee[2];      // 膝关节角度
    float qd_abad[2];     // 髋外展关节角速度
    float qd_hip[2];      // 髋关节角速度
    float qd_knee[2];     // 膝关节角速度
    int32_t flags[2];     // 每条腿的状态标志位（软限位等）
    int32_t checksum;     // 按 32-bit 字计算的 XOR 校验
};

// 主机下发给腿控板的 66 个 16-bit 字命令帧，约 132 字节。
struct spi_command_t {
    float q_des_abad[2];  // 目标角度：髋外展
    float q_des_hip[2];   // 目标角度：髋关节
    float q_des_knee[2];  // 目标角度：膝关节
    float qd_des_abad[2]; // 目标角速度：髋外展
    float qd_des_hip[2];  // 目标角速度：髋关节
    float qd_des_knee[2]; // 目标角速度：膝关节
    float kp_abad[2];     // 比例增益：髋外展
    float kp_hip[2];      // 比例增益：髋关节
    float kp_knee[2];     // 比例增益：膝关节
    float kd_abad[2];     // 微分增益：髋外展
    float kd_hip[2];      // 微分增益：髋关节
    float kd_knee[2];     // 微分增益：膝关节
    float tau_abad_ff[2]; // 前馈力矩：髋外展
    float tau_hip_ff[2];  // 前馈力矩：髋关节
    float tau_knee_ff[2]; // 前馈力矩：膝关节
    int32_t flags[2];     // 主机控制标志位（含力矩使能等）
    int32_t checksum;     // 校验值，主机按 32-bit 字 XOR 计算
};

// 单个关节的目标控制量。
struct joint_control {
    float p_des, v_des, kp, kd, t_ff;
};

// 单个关节的当前状态。
struct joint_state {
    float p, v, t;
};

// 单条腿的状态（a: 髋外展，h: 髋，k: 膝）。
struct leg_state {
    joint_state a, h, k;
};

// 单条腿的控制量。
struct leg_control {
    joint_control a, h, k;
};
#endif
