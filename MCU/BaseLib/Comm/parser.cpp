#include "parser.hpp"

#include <utility>

namespace comm {

PackParser::PackParser(Comm& comm)
    : comm_(&comm)
{
    comm_->register_recv_callback([this](void* data, int size) {
        on_received_pack(data, size);
    });
}

void PackParser::register_cmd_callback(uint32_t cmd, std::function<void(const Command* cmd)> func)
{
    callbacks_[cmd] = std::move(func);
}

void PackParser::on_received_pack(void* data, int size)
{
    constexpr int header_size = static_cast<int>(sizeof(Command));
    if (data == nullptr || size < header_size) {
        return;
    }

    const auto* pack = static_cast<const Command*>(data);
    if (pack->size != static_cast<uint32_t>(size - header_size)) {
        return;
    }

    const auto iter = callbacks_.find(pack->cmd);
    if (iter != callbacks_.end() && iter->second) {
        iter->second(pack);
    }
}

void PackParser::unregister_callback(uint32_t cmd)
{
    callbacks_.erase(cmd);
}

} // namespace comm
