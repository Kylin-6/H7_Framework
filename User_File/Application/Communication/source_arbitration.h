#ifndef RM_SOURCE_ARBITRATION_H
#define RM_SOURCE_ARBITRATION_H

#include "input_state.h"

struct InputDecision
{
    ChassisCmd chassis{};
    GimbalCmd gimbal{};
    ShootCmd shoot{};
    InputSource source = InputSource::Remote;
    bool armed = false;
};

/** 固定优先级：Remote 安全许可 → 显式选中且新鲜的控制来源 → 可选 Vision 云台瞄准。 */
InputDecision SourceArbitration_Resolve(const InputState &state);

#endif
