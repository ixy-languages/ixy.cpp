#include "ixy/mempool.hpp"

#include "ixy/packet.hpp"
#include "log.hpp"
#include "memory.hpp"

#include <cassert>

namespace ixy {

auto Mempool::allocate(uint32_t num_entries, uint32_t entry_size) -> std::shared_ptr<Mempool> {
    entry_size = entry_size ? entry_size : 2048;
    // require entries that neatly fit into the page size, this makes the memory pool much easier
    // otherwise our base_addr + index * size formula would be wrong because we can't cross a page-boundary
    if (HUGE_PAGE_SIZE % entry_size) {
        error("entry size must be a divisor of the huge page size " << HUGE_PAGE_SIZE);
    }

    struct dma_memory mem = memory_allocate_dma(num_entries * entry_size, false);
    auto pool = Mempool{static_cast<uint8_t *>(mem.virt), entry_size, num_entries, std::vector<uint64_t>(),
                        std::stack<uint64_t>()};

    for (uint32_t i = 0; i < num_entries; i++) {
        auto addr = virt_to_phys(static_cast<uint8_t *>(mem.virt) + (i * entry_size));
        pool.physical_addr.push_back(addr);
        pool.free_stack.push(i);
    }

    return std::make_shared<Mempool>(std::move(pool));
}

auto Mempool::alloc_pkt(std::shared_ptr<Mempool> &pool, uint32_t packet_size) -> std::optional<Packet> {
    if (packet_size <= pool->buf_size) {
        auto buf = pool->alloc_buf();

        if (buf.has_value()) {
            auto id = buf.value();

            return Packet(pool->get_virt_addr(id), pool->get_phys_addr(id), packet_size, pool, id);
        }
    }

    return {};
}

auto Mempool::alloc_pkt_batch(std::shared_ptr<Mempool> &pool, std::deque<Packet> &buffer, uint32_t num_packets,
                              uint32_t packet_size) -> uint32_t {
    uint32_t allocated = 0;

    while (auto p = alloc_pkt(pool, packet_size)) {
        buffer.push_back(std::move(p.value()));

        allocated += 1;
        if (allocated >= num_packets) {
            break;
        }
    }

    return allocated;
}

auto Mempool::alloc_buf() -> std::optional<uint64_t> {
    if (this->free_stack.empty()) {
        return {};
    }

    uint64_t buf = this->free_stack.top();
    this->free_stack.pop();
    return buf;
}

void Mempool::free_buf(uint64_t id) {
    assert(id < this->num_entries);
    this->free_stack.push(id);
}

auto Mempool::get_virt_addr(uint64_t id) -> uint8_t * {
    assert(id < this->num_entries);
    return (this->base_addr + id * this->buf_size);
}

auto Mempool::get_phys_addr(uint64_t id) -> uint64_t {
    return this->physical_addr.at(id);
}

}  // namespace ixy
