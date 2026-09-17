/**
 * @file alg_dob.h
 * @author zzm
 * @brief 一阶名义模型的Q滤波扰动观测器
 */

#ifndef __ALG_DOB_H
#define __ALG_DOB_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 一阶名义模型DOB
 * @details 模型为 Time_Constant * dy/dt + y = Gain * (u + d)。
 * 使用零阶保持离散模型和两个相同实极点的Q滤波器。
 * 输出为上一采样区间等效输入扰动的滤波估计。
 * Init成功后使用, 调用方保证固定采样周期, y[k]与上一周期实际输入u[k-1]对齐。
 */
class Class_DOB_First_Order
{
public:
    bool Init(float __Gain, float __Time_Constant, float __Filter_Frequency, float __D_T = 0.001f);

    void Reset(float __Now = 0.0f);

    inline float Get_Out() const;

    inline void Set_Now(float __Now);

    inline void Set_Input(float __Input);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    float Gain_Inverse = 0.0f;
    float Filter_Alpha = 0.0f;
    float Difference_Gain = 0.0f;

    float Now = 0.0f;
    float Input = 0.0f;
    float Pre_Now = 0.0f;
    float Filter_Out = 0.0f;
    float Out = 0.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_DOB_First_Order::Get_Out() const
{
    return (Out);
}

inline void Class_DOB_First_Order::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @param __Input 上一采样周期限幅后实际施加的输入, 与名义模型输入同单位
 */
inline void Class_DOB_First_Order::Set_Input(float __Input)
{
    Input = __Input;
}

#endif
