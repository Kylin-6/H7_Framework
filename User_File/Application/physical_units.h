#ifndef APPLICATION_PHYSICAL_UNITS_H
#define APPLICATION_PHYSICAL_UNITS_H

constexpr float kPiRad = 3.14159265358979323846f;

// 仅用于机械标定参数和外部角度制接口的入口转换。
constexpr float DegToRad(float degree)
{
    return degree * (kPiRad / 180.0f);
}

#endif
