#ifndef MESSAGE_CENTER_OUTPUT_H
#define MESSAGE_CENTER_OUTPUT_H

#include "topic.h"

/** A fixed publisher binding. The owner of context must outlive this handle. */
template<typename T>
class Output
{
public:
    using PublishFn = void (*)(void *, const T &);

    Output() = default;
    Output(void *context, PublishFn publish) : context_(context), publish_(publish) {}

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
