/**
 * @file alg_fuzzy.cpp
 * @author zzm
 * @brief 双输入零阶Sugeno模糊推理
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_fuzzy.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 绑定严格递增的节点及完整规则表, 不复制数组、不分配内存
 * @return 配置有效时返回true; 失败保留原配置
 */
bool Class_Fuzzy_Sugeno::Init(const Struct_Fuzzy_Sugeno_Config &__Config)
{
    if (__Config.Rule_Table == NULL || __Config.Output_Count == 0 ||
        !Check_Nodes(__Config.Input_1_Nodes, __Config.Input_1_Count) ||
        !Check_Nodes(__Config.Input_2_Nodes, __Config.Input_2_Count))
    {
        return (false);
    }

    uint64_t length = (uint64_t)__Config.Input_1_Count *
                      __Config.Input_2_Count * __Config.Output_Count;
    if (length != __Config.Rule_Table_Length)
    {
        return (false);
    }
    for (uint32_t i = 0; i < __Config.Rule_Table_Length; i++)
    {
        if (Basic_Math_Is_Invalid_Float(__Config.Rule_Table[i]))
        {
            return (false);
        }
    }

    Config = __Config;
    return (true);
}

/**
 * @brief 对两个输入执行一次推理
 * @param __Out 调用方提供的Output_Count个float, 不得与节点或规则表重叠
 * @return 未初始化、输出指针为空或输入无效时返回false且不写输出
 */
bool Class_Fuzzy_Sugeno::Calculate(float __Input_1, float __Input_2, float *__Out) const
{
    if (Config.Rule_Table == NULL || __Out == NULL ||
        Basic_Math_Is_Invalid_Float(__Input_1) ||
        Basic_Math_Is_Invalid_Float(__Input_2))
    {
        return (false);
    }

    float ratio_1;
    float ratio_2;
    uint16_t index_1 = Find_Interval(Config.Input_1_Nodes, Config.Input_1_Count,
                                    __Input_1, &ratio_1);
    uint16_t index_2 = Find_Interval(Config.Input_2_Nodes, Config.Input_2_Count,
                                    __Input_2, &ratio_2);
    uint32_t offset = ((uint32_t)index_1 * Config.Input_2_Count + index_2) *
                      Config.Output_Count;
    uint32_t stride = (uint32_t)Config.Input_2_Count * Config.Output_Count;
    const float *rule_00 = Config.Rule_Table + offset;
    const float *rule_01 = rule_00 + Config.Output_Count;
    const float *rule_10 = rule_00 + stride;
    const float *rule_11 = rule_10 + Config.Output_Count;

    for (uint16_t i = 0; i < Config.Output_Count; i++)
    {
        float low = (1.0f - ratio_1) * rule_00[i] + ratio_1 * rule_10[i];
        float high = (1.0f - ratio_1) * rule_01[i] + ratio_1 * rule_11[i];
        __Out[i] = (1.0f - ratio_2) * low + ratio_2 * high;
    }
    return (true);
}

bool Class_Fuzzy_Sugeno::Check_Nodes(const float *__Nodes, uint16_t __Count)
{
    if (__Nodes == NULL || __Count < 2)
    {
        return (false);
    }
    for (uint16_t i = 0; i < __Count; i++)
    {
        if (Basic_Math_Is_Invalid_Float(__Nodes[i]))
        {
            return (false);
        }
        if (i > 0)
        {
            float width = __Nodes[i] - __Nodes[i - 1];
            if (Basic_Math_Is_Invalid_Float(width) || width <= 0.0f)
            {
                return (false);
            }
        }
    }
    return (true);
}

uint16_t Class_Fuzzy_Sugeno::Find_Interval(const float *__Nodes, uint16_t __Count,
                                         float __Input, float *__Ratio)
{
    if (__Input <= __Nodes[0])
    {
        *__Ratio = 0.0f;
        return (0);
    }
    if (__Input >= __Nodes[__Count - 1])
    {
        *__Ratio = 1.0f;
        return (__Count - 2);
    }

    uint16_t low = 0;
    uint16_t high = __Count - 1;
    while (high - low > 1)
    {
        uint16_t mid = low + (high - low) / 2;
        if (__Input < __Nodes[mid])
        {
            high = mid;
        }
        else
        {
            low = mid;
        }
    }
    *__Ratio = (__Input - __Nodes[low]) / (__Nodes[high] - __Nodes[low]);
    return (low);
}
