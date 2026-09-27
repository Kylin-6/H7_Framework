#ifndef MESSAGE_CENTER_OUTPUT_H
#define MESSAGE_CENTER_OUTPUT_H

#include "topic.h"

/** 无堆分配、无虚函数的输出句柄；不拥有 context，绑定对象须比句柄活得更久。 */
template<typename T>
class Output
{
public:
    using PublishFn = void (*)(void *, const T &);

    Output() = default;
    Output(void *context, PublishFn publish) : context_(context), publish_(publish) {}

    /** 调用方初始化时必须检查；未绑定的 Publish 当前为无操作，不是有效配置。 */
    bool IsBound() const { return context_ != nullptr && publish_ != nullptr; }

    void Publish(const T &data) const
    {
        if (IsBound())
        {
            publish_(context_, data);
        }
    }

private:
    void *context_ = nullptr;
    PublishFn publish_ = nullptr;
};

template<typename T>
class LocalPublisher
{
public:
    explicit LocalPublisher(Topic<T> &topic) : publisher_(topic) {}

    Output<T> Bind()
    {
        return {this, [](void *context, const T &data) {
            static_cast<LocalPublisher *>(context)->publisher_.Publish(data);
        }};
    }

private:
    Publisher<T> publisher_;
};

#endif
