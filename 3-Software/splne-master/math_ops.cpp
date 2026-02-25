
#include "math_ops.h"


// 返回 x 和 y 中的较大值（浮点）。
float fmaxf(float x, float y){
    return (((x)>(y))?(x):(y));
    }

// 返回 x 和 y 中的较小值（浮点）。
float fminf(float x, float y){
    return (((x)<(y))?(x):(y));
    }

// 返回 x、y、z 中的最大值。
float fmaxf3(float x, float y, float z){
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

// 返回 x、y、z 中的最小值。
float fminf3(float x, float y, float z){
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
    
// 将向量 (x, y) 的模长限制在 limit 以内，超限则按比例缩放。
void limit_norm(float *x, float *y, float limit){
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }


// 按范围与位宽，将浮点转换为无符号整型编码。
int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
// 按范围与位宽，将无符号整型解码为浮点。
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
