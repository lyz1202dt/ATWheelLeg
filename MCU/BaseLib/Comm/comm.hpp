#pragma once

#include <functional>

namespace comm {

class Comm {
public:
    using ReceiveCallback = std::function<void(void* data, int size)>;

    Comm() = default;
    virtual ~Comm() = default;

    virtual bool init() { return false; }
    virtual void register_recv_callback(ReceiveCallback callback) { (void)callback; }
    virtual bool send_pack(void* data, int size)
    {
        (void)data;
        (void)size;
        return false;
    }
};

} // namespace comm
