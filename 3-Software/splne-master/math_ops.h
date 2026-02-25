#ifndef MATH_OPS_H
#define MATH_OPS_H

// math_ops.h：为 MCU 平台提供常用的数学小工具，避免依赖标准库的重实现。

#define PI 3.14159265359f  // 圆周率常量

#include "math.h"

// 返回 x 与 y 中的较大值。
float fmaxf(float x, float y);

// 返回 x 与 y 中的较小值。
float fminf(float x, float y);

// 返回 x、y、z 中的最大值。
float fmaxf3(float x, float y, float z);

// 返回 x、y、z 中的最小值。
float fminf3(float x, float y, float z);

// 将向量 (x, y) 的模长限制在 limit 以内（若超限则按比例缩放）。
void limit_norm(float *x, float *y, float limit);

// 按给定范围与位宽，将浮点转换为无符号整型。
int float_to_uint(float x, float x_min, float x_max, int bits);

// 按给定范围与位宽，将无符号整型还原为浮点。
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif
