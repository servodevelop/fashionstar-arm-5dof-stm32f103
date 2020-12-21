#ifndef __COMMON_H
#define __COMMON_H

#define bool uint8_t
#define true 1
#define false 0

// 求最大值
#define max(x, y) (((x) > (y)) ? (x):(y))

#define pi 3.1415926

// 角度转弧度
#define radians(theta) ((theta)/180.0 * pi)

// 弧度转角度
#define degrees(theta) ((theta)/pi*180.0)

#endif
