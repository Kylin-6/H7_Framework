#ifndef BOARD_REMOTE_PUBLISHER_H
#define BOARD_REMOTE_PUBLISHER_H

#include "board_transport.h"
#include "output.h"

template<typename T>
class RemotePublisher;

template<>
class RemotePublisher<ChassisCmd>
{
public:
    Output<ChassisCmd> Bind()
    {
        return {this, [](void *, const ChassisCmd &data) {
            BoardTransport_SendChassis(data);
        }};
    }
};

#endif
