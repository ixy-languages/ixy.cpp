#ifndef IXY_PACKET_HPP
#define IXY_PACKET_HPP

#include <iostream>
#include <memory>

#include "mempool.hpp"

namespace ixy {

class Packet {
    friend class IxgbeDevice;

    friend class Mempool;

public:
    // remove copy constructor
    Packet(const Packet &) = delete;

    // remove copy assignment
    auto operator=(const Packet &) -> Packet & = delete;

    // re-add implicitly removed move constructor
    Packet(Packet &&old) noexcept: addr_virt{old.addr_virt},
                                   addr_phys{old.addr_phys},
                                   len{old.len},
                                   pool{old.pool},
                                   pool_entry{old.pool_entry} {
        old.pool = nullptr;
    }

    // re-add implicitly removed move assignment
    auto operator=(Packet &&old) noexcept -> Packet & {
        addr_virt = old.addr_virt;
        addr_phys = old.addr_phys;
        len = old.len;
        pool = old.pool;
        pool_entry = old.pool_entry;

        old.pool = nullptr;

        return *this;
    }

    ~Packet() {
        if (pool) {
            pool->free_buf(pool_entry);
        }
    }

    [[nodiscard]] auto at(uint64_t n) -> uint8_t & {
        if (n >= len) {
            throw std::out_of_range{"tried to access byte " + std::to_string(n) + " out of " + std::to_string(len) + " bytes"};
        }

        return *(addr_virt + n);
    }

    [[nodiscard]] auto get_virt_addr() -> uint8_t * {
        return addr_virt;
    }

    [[nodiscard]] auto get_phys_addr() -> uint64_t {
        return addr_phys;
    }

    [[nodiscard]] auto size() const -> uint64_t {
        return len;
    }

private:
    Packet(uint8_t *addr_virt, uint64_t addr_phys, uint64_t len, Mempool *pool, uint64_t pool_entry) :
            addr_virt{addr_virt},
            addr_phys{addr_phys},
            len{len},
            pool{pool},
            pool_entry{pool_entry} {}

    uint8_t *addr_virt;
    uint64_t addr_phys;
    uint64_t len;
    Mempool *pool;
    uint64_t pool_entry;
};

}  // namespace ixy

#endif //IXY_PACKET_HPP
