#pragma once

#include "FreeRTOS.h"
#include "comm.hpp"
#include "pack.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <unordered_map>

namespace comm {

class PackParser {
public:
    static constexpr int kSendBufferSize = 128;

    explicit PackParser(Comm& comm);
    PackParser(const PackParser&) = delete;
    PackParser& operator=(const PackParser&) = delete;

    template <typename T>
    bool send_command(uint32_t cmd, const T& data)
    {
        constexpr uint32_t data_size = sizeof(T);
        static_assert(data_size + sizeof(Command) <= kSendBufferSize,
                      "PackParser single packet is too large");

        auto* pack = reinterpret_cast<Command*>(send_buffer_);
        pack->cmd = cmd;
        pack->size = data_size;
        std::memcpy(pack->data, &data, data_size);
        return comm_->send_pack(pack, sizeof(Command) + data_size);
    }

    void on_received_pack(void* data, int size);
    void register_cmd_callback(uint32_t cmd, std::function<void(const Command* cmd)> func);
    void unregister_callback(uint32_t cmd);

private:
    std::unordered_map<uint32_t, std::function<void(const Command* cmd)>> callbacks_;
    uint8_t send_buffer_[kSendBufferSize] = {};
    Comm* comm_ = nullptr;
};

} // namespace comm
