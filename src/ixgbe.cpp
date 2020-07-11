#include <cstring>
#include <cassert>
#include <thread>

#include "ixgbe.hpp"
#include "ixgbe_type.hpp"
#include "memory.hpp"
#include "ixy/stats.hpp"

namespace ixy {

const std::string IxgbeDevice::driver_name = "ixgbe";

const unsigned int NUM_RX_QUEUE_ENTRIES = 512;
const unsigned int NUM_TX_QUEUE_ENTRIES = 512;

const unsigned int PKT_BUF_ENTRY_SIZE = 2048;
const unsigned int MIN_MEMPOOL_ENTRIES = 4096;

const unsigned int TX_CLEAN_BATCH = 32;

constexpr auto wrap_ring(uint16_t index, uint16_t ring_size) -> uint16_t {
    return static_cast<uint16_t>((index + 1) & ((ring_size) - 1));
}

// see section 4.6.3
void IxgbeDevice::reset_and_init() {
    info("resetting device " << pci_addr);

    // section 4.6.3.1 - disable all interrupts
    set_reg32(IXGBE_EIMC, 0x7FFFFFFF);

    // section 4.6.3.2
    set_reg32(IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
    wait_clear_reg32(IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // section 4.6.3.1 - disable interrupts again after reset
    set_reg32(IXGBE_EIMC, 0x7FFFFFFF);

    info("initializing device " << pci_addr);

    auto mac = get_mac_addr();

    info("mac address: " << std::hex << unsigned(mac[0]) << ":" << unsigned(mac[1]) << ":" << unsigned(mac[2])
                         << ":" << unsigned(mac[3]) << ":" << unsigned(mac[4]) << ":" << unsigned(mac[5]));

    // section 4.6.3 - wait for EEPROM auto read completion
    wait_set_reg32(IXGBE_EEC, IXGBE_EEC_ARD);

    // section 4.6.3 - wait for DMA initialization done (RDRXCTL.DMAIDONE)
    wait_set_reg32(IXGBE_RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);

    // section 4.6.4 - initialize link (auto negotiation)
    init_link();

    // section 4.6.5 - statistical counters
    // reset-on-read registers, just read them once
    reset_stats();

    // section 4.6.7 - init rx
    init_rx();

    // section 4.6.8 - init tx
    init_tx();

    // enables queues after initializing everything
    for (uint16_t i = 0; i < num_rx_queues; i++) {
        start_rx_queue(i);
    }
    for (uint16_t i = 0; i < num_tx_queues; i++) {
        start_tx_queue(i);
    }

    // skip last step from 4.6.3 - don't want interrupts
    // finally, enable promisc mode by default, it makes testing less annoying
    set_promisc(true);

    // wait for some time for the link to come up
    wait_for_link();
}

auto IxgbeDevice::get_driver_name() -> const std::string & {
    return driver_name;
}

auto IxgbeDevice::get_pci_addr() -> const std::string & {
    return pci_addr;
}

auto IxgbeDevice::get_mac_addr() -> std::array<uint8_t, 6> {
    auto low = get_reg32(IXGBE_RAL(0));
    auto high = get_reg32(IXGBE_RAH(0));

    return {
            static_cast<uint8_t>(low),
            static_cast<uint8_t>(low >> 8u),
            static_cast<uint8_t>(low >> 16u),
            static_cast<uint8_t>(low >> 24u),
            static_cast<uint8_t>(high),
            static_cast<uint8_t>(high >> 8u),
    };
}

void IxgbeDevice::set_mac_addr(std::array<uint8_t, 6> &mac) {
    auto low = mac.at(0) + (mac.at(1) << 8u) + (mac.at(2) << 16u) + (mac.at(3) << 24u);
    auto high = mac.at(4) + (mac.at(5) << 8u);

    set_reg32(IXGBE_RAL(0), low);
    set_reg32(IXGBE_RAH(0), high);
}

void IxgbeDevice::init_rx() {
    // make sure that rx is disabled while re-configuring it
    // the datasheet also wants us to disable some crypto-offloading related rx paths (but we don't care about them)
    clear_flags32(IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
    // no fancy dcb or vt, just a single 128kb packet buffer for us
    set_reg32(IXGBE_RXPBSIZE(0), IXGBE_RXPBSIZE_128KB);
    for (int i = 1; i < 8; i++) {
        set_reg32(IXGBE_RXPBSIZE(i), 0);
    }

    // always enable CRC offloading
    set_flags32(IXGBE_HLREG0, IXGBE_HLREG0_RXCRCSTRP);
    set_flags32(IXGBE_RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

    // accept broadcast packets
    set_flags32(IXGBE_FCTRL, IXGBE_FCTRL_BAM);

    // per-queue config, same for all queues
    for (uint16_t i = 0; i < num_rx_queues; i++) {
        debug("initializing rx queue " << i);
        // enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
        set_reg32(IXGBE_SRRCTL(i),
                  (get_reg32(IXGBE_SRRCTL(i)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
        // drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
        // a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
        set_flags32(IXGBE_SRRCTL(i), IXGBE_SRRCTL_DROP_EN);
        // setup descriptor ring, see section 7.1.9
        uint32_t ring_size_bytes = NUM_RX_QUEUE_ENTRIES * sizeof(union ixgbe_adv_rx_desc);
        dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
        // neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
        memset(mem.virt, -1, ring_size_bytes);
        // tell the device where it can write to (its iova, so its view)
        set_reg32(IXGBE_RDBAL(i), static_cast<uint32_t>(mem.phy & 0xFFFFFFFFull));
        set_reg32(IXGBE_RDBAH(i), static_cast<uint32_t>(mem.phy >> 32u));
        set_reg32(IXGBE_RDLEN(i), ring_size_bytes);
        debug("rx ring " << i << " phy addr:  0x" << std::hex << mem.phy);
        debug("rx ring " << i << " virt addr: 0x" << std::hex << (uintptr_t) mem.virt);
        // set ring to empty at start
        set_reg32(IXGBE_RDH(i), 0);
        set_reg32(IXGBE_RDT(i), 0);
        // private data for the driver, 0-initialized

        // 2048 as pktbuf size is strictly speaking incorrect:
        // we need a few headers (1 cacheline), so there's only 1984 bytes left for the device
        // but the 82599 can only handle sizes in increments of 1 kb; but this is fine since our max packet size
        // is the default MTU of 1518
        // this has to be fixed if jumbo frames are to be supported
        // mempool should be >= the number of rx and tx descriptors for a forwarding application
        auto mempool_size = NUM_RX_QUEUE_ENTRIES + NUM_TX_QUEUE_ENTRIES;
        auto pool = Mempool::allocate(mempool_size < MIN_MEMPOOL_ENTRIES ? MIN_MEMPOOL_ENTRIES : mempool_size,
                                      PKT_BUF_ENTRY_SIZE);

        ixgbe_rx_queue queue{static_cast<union ixgbe_adv_rx_desc *>(mem.virt), pool, NUM_RX_QUEUE_ENTRIES, 0,
                             std::vector<uint64_t>()};
        rx_queues.push_back(queue);
    }

    // last step is to set some magic bits mentioned in the last sentence in 4.6.7
    set_flags32(IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);
    // this flag probably refers to a broken feature: it's reserved and initialized as '1' but it must be set to '0'
    // there isn't even a constant in ixgbe_types.h for this flag
    for (uint16_t i = 0; i < num_rx_queues; i++) {
        clear_flags32(IXGBE_DCA_RXCTRL(i), 1u << 12u);
    }

    // start RX
    set_flags32(IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
}

void IxgbeDevice::init_tx() {
    // crc offload and small packet padding
    set_flags32(IXGBE_HLREG0, IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN);

    // set default buffer size allocations
    // see also: section 4.6.11.3.4, no fancy features like DCB and VTd
    set_reg32(IXGBE_TXPBSIZE(0), IXGBE_TXPBSIZE_40KB);
    for (int i = 1; i < 8; i++) {
        set_reg32(IXGBE_TXPBSIZE(i), 0);
    }
    // required when not using DCB/VTd
    set_reg32(IXGBE_DTXMXSZRQ, 0xFFFF);
    clear_flags32(IXGBE_RTTDCS, IXGBE_RTTDCS_ARBDIS);

    // per-queue config for all queues
    for (uint16_t i = 0; i < num_tx_queues; i++) {
        debug("initializing tx queue " << i);

        // setup descriptor ring, see section 7.1.9
        auto ring_size_bytes = NUM_TX_QUEUE_ENTRIES * sizeof(union ixgbe_adv_tx_desc);
        dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
        memset(mem.virt, -1, ring_size_bytes);
        // tell the device where it can write to (its iova, so its view)
        set_reg32(IXGBE_TDBAL(i), static_cast<uint32_t>(mem.phy & 0xFFFFFFFFull));
        set_reg32(IXGBE_TDBAH(i), static_cast<uint32_t>(mem.phy >> 32u));
        set_reg32(IXGBE_TDLEN(i), ring_size_bytes);
        debug("tx ring " << i << " phy addr:  0x" << std::hex << mem.phy);
        debug("tx ring " << i << " virt addr: 0x" << std::hex << (uintptr_t) mem.virt);

        // descriptor writeback magic values, important to get good performance and low PCIe overhead
        // see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
        // we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
        auto txdctl = get_reg32(IXGBE_TXDCTL(i));
        // there are no defines for this in ixgbe_type.hpp for some reason
        // pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
        txdctl &= ~(0x3Fu | (0x3Fu << 8u) | (0x3Fu << 16u)); // clear bits
        txdctl |= (36u | (8u << 8u) | (4u << 16u)); // from DPDK
        set_reg32(IXGBE_TXDCTL(i), txdctl);

        // private data for the driver, 0-initialized
        ixgbe_tx_queue queue{static_cast<union ixgbe_adv_tx_desc *>(mem.virt), nullptr, NUM_TX_QUEUE_ENTRIES, 0, 0,
                             std::deque<uint64_t>()};
        tx_queues.push_back(queue);
    }

    // final step: enable DMA
    set_reg32(IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);
}

void IxgbeDevice::start_rx_queue(uint16_t queue_id) {
    debug("starting rx queue " << queue_id);
    auto queue = &rx_queues.at(queue_id);

    if (queue->num_entries & (queue->num_entries - 1)) {
        error("number of queue entries must be a power of 2");
    }

    for (int i = 0; i < queue->num_entries; i++) {
        auto buf = queue->mempool->alloc_buf();

        volatile union ixgbe_adv_rx_desc *rxd = queue->descriptors + i;
        rxd->read.pkt_addr = queue->mempool->get_phys_addr(buf);
        rxd->read.hdr_addr = 0;

        queue->bufs_in_use.push_back(buf);
    }

    // enable queue and wait if necessary
    set_flags32(IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
    wait_set_reg32(IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
    // rx queue starts out full
    set_reg32(IXGBE_RDH(queue_id), 0);
    // was set to 0 before in the init function
    set_reg32(IXGBE_RDT(queue_id), queue->num_entries - 1);
}

void IxgbeDevice::start_tx_queue(uint16_t queue_id) {
    debug("starting tx queue " << queue_id);
    struct ixgbe_tx_queue *queue = &tx_queues.at(queue_id);
    if (queue->num_entries & unsigned(queue->num_entries - 1)) {
        error("number of queue entries must be a power of 2");
    }
    // tx queue starts out empty
    set_reg32(IXGBE_TDH(queue_id), 0);
    set_reg32(IXGBE_TDT(queue_id), 0);
    // enable queue and wait if necessary
    set_flags32(IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
    wait_set_reg32(IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
}

auto IxgbeDevice::rx_batch(uint16_t queue_id, std::deque<Packet> &buffer, uint32_t packets) -> uint32_t {
    uint32_t received_packets;

    struct ixgbe_rx_queue *queue = &rx_queues.at(queue_id);

    auto rx_index = queue->rx_index;
    auto last_rx_index = rx_index;

    for (received_packets = 0; received_packets < packets; received_packets++) {
        volatile union ixgbe_adv_rx_desc *desc_ptr = queue->descriptors + rx_index;
        auto status = desc_ptr->wb.upper.status_error;
        if (status & IXGBE_RXDADV_STAT_DD) {
            if (!(status & IXGBE_RXDADV_STAT_EOP)) {
                error("multi-segment packets are not supported - increase buffer size or decrease MTU");
            }

            auto buf = queue->bufs_in_use.at(rx_index);

            auto new_buf = queue->mempool->alloc_buf();
            queue->bufs_in_use.at(rx_index) = new_buf;

            auto pool = queue->mempool;

            auto p = Packet(pool->get_virt_addr(buf), pool->get_phys_addr(buf), desc_ptr->wb.upper.length, pool,
                            buf);

            buffer.push_back(std::move(p));

            // reset the descriptor
            desc_ptr->read.pkt_addr = pool->get_phys_addr(new_buf);
            desc_ptr->read.hdr_addr = 0; // this resets the flags

            // want to read the next one in the next iteration, but we still need the last/current to update RDT later
            last_rx_index = rx_index;
            rx_index = wrap_ring(rx_index, queue->num_entries);
        } else {
            break;
        }
    }

    if (rx_index != last_rx_index) {
        // tell hardware that we are done
        // this is intentionally off by one, otherwise we'd set RDT=RDH if we are receiving faster than packets are coming in
        // RDT=RDH means queue is full
        set_reg32(IXGBE_RDT(queue_id), last_rx_index);
        queue->rx_index = rx_index;
    }

    return received_packets;
}

auto IxgbeDevice::tx_batch(uint16_t queue_id, std::deque<Packet> &buffer) -> uint32_t {
    auto sent_packets = 0;

    struct ixgbe_tx_queue *queue = &tx_queues.at(queue_id);

    clean_tx_queue(queue);

    auto cur_index = queue->tx_index;
    auto clean_index = queue->clean_index;

    if (!queue->mempool && !buffer.empty()) {
        queue->mempool = buffer.front().pool;
    }

    while (!buffer.empty()) {
        auto &p = buffer.front();

        auto next_index = wrap_ring(cur_index, queue->num_entries);

        if (clean_index == next_index) {
            // tx queue of device is full
            break;
        }

        volatile union ixgbe_adv_tx_desc *txd = queue->descriptors + queue->tx_index;
        // NIC reads from here
        txd->read.buffer_addr = p.get_phys_addr();
        // always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
        txd->read.cmd_type_len =
                static_cast<u32>(IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS |
                                 IXGBE_ADVTXD_DCMD_DEXT |
                                 IXGBE_ADVTXD_DTYP_DATA | p.size());
        // no fancy offloading stuff - only the total payload length
        // implement offloading flags here:
        // 	* ip checksum offloading is trivial: just set the offset
        // 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
        txd->read.olinfo_status = static_cast<u32>(p.size() << IXGBE_ADVTXD_PAYLEN_SHIFT);

        queue->bufs_in_use.push_back(p.pool_entry);

        p.pool.reset();

        queue->tx_index = wrap_ring(queue->tx_index, queue->num_entries);

        buffer.pop_front();

        cur_index = next_index;
        sent_packets += 1;
    }

    set_reg32(IXGBE_TDT(queue_id), queue->tx_index);

    return sent_packets;
}

void IxgbeDevice::read_stats(struct device_stats &stats) {
    stats.dev = *this;
    stats.rx_pkts += get_reg32(IXGBE_GPRC);
    stats.tx_pkts += get_reg32(IXGBE_GPTC);
    stats.rx_bytes += get_reg32(IXGBE_GORCL) + ((static_cast<uint64_t>(get_reg32(IXGBE_GORCH))) << 32u);
    stats.tx_bytes += get_reg32(IXGBE_GOTCL) + ((static_cast<uint64_t>(get_reg32(IXGBE_GOTCH))) << 32u);
}

void IxgbeDevice::reset_stats() {
    get_reg32(IXGBE_GPRC);
    get_reg32(IXGBE_GPTC);
    get_reg32(IXGBE_GORCL);
    get_reg32(IXGBE_GORCH);
    get_reg32(IXGBE_GOTCL);
    get_reg32(IXGBE_GOTCH);
}

void IxgbeDevice::set_promisc(bool enabled) {
    if (enabled) {
        info("enabling promisc mode");
        set_flags32(IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
    } else {
        info("disabling promisc mode");
        clear_flags32(IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
    }
}

void IxgbeDevice::init_link() {
    // should already be set by the eeprom config, maybe we shouldn't override it here to support weirdo nics?
    set_reg32(IXGBE_AUTOC, (get_reg32(IXGBE_AUTOC) & ~IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL);
    set_reg32(IXGBE_AUTOC, (get_reg32(IXGBE_AUTOC) & ~IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI);
    // negotiate link
    set_flags32(IXGBE_AUTOC, IXGBE_AUTOC_AN_RESTART);
    // datasheet wants us to wait for the link here, but we can continue and wait afterwards
}

void IxgbeDevice::wait_for_link() {
    info("waiting for link...");
    auto max_wait = 10000;    // 10 seconds in milliseconds
    auto poll_interval = 10; // milliseconds
    while (!(get_link_speed()) && max_wait > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval));
        max_wait -= poll_interval;
    }
    info("link speed is " << std::dec << get_link_speed() << " Mbit/s");
}

auto IxgbeDevice::get_link_speed() -> uint32_t {
    auto links = get_reg32(IXGBE_LINKS);
    auto speed = 0;

    if ((links & IXGBE_LINKS_UP)) {
        switch (links & IXGBE_LINKS_SPEED_82599) {
            case IXGBE_LINKS_SPEED_100_82599:
                speed = 100;
                break;
            case IXGBE_LINKS_SPEED_1G_82599:
                speed = 1000;
                break;
            case IXGBE_LINKS_SPEED_10G_82599:
                speed = 10000;
                break;
            default:
                break;
        }
    }

    return speed;
}

inline auto IxgbeDevice::get_reg32(uint64_t reg) -> uint32_t {
    assert(reg <= len - 4);

    __asm__ volatile ("" : : : "memory");
    return *(reinterpret_cast<volatile uint32_t *>(addr + reg));
}

inline void IxgbeDevice::set_reg32(uint64_t reg, uint32_t value) {
    assert(reg <= len - 4);

    __asm__ volatile ("" : : : "memory");
    *(reinterpret_cast<volatile uint32_t *>(addr + reg)) = value;
}

inline void IxgbeDevice::set_flags32(uint64_t reg, uint32_t flags) {
    set_reg32(reg, get_reg32(reg) | flags);
}

inline void IxgbeDevice::clear_flags32(uint64_t reg, uint32_t flags) {
    set_reg32(reg, get_reg32(reg) & ~flags);
}

inline void IxgbeDevice::wait_clear_reg32(uint64_t reg, uint32_t mask) {
    __asm__ volatile ("" : : : "memory");
    uint32_t cur;
    while (cur = *(reinterpret_cast<volatile uint32_t *>(addr + reg)), (cur & mask) != 0) {
        debug("waiting for flags " << mask << " in register " << reg << " to clear, current value " << cur);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        __asm__ volatile ("" : : : "memory");
    }
}

inline void IxgbeDevice::wait_set_reg32(uint64_t reg, uint32_t mask) {
    __asm__ volatile ("" : : : "memory");
    uint32_t cur;
    while (cur = *(reinterpret_cast<volatile uint32_t *>(addr + reg)), (cur & mask) != mask) {
        debug("waiting for flags " << mask << " in register " << reg << ", current value " << cur);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        __asm__ volatile ("" : : : "memory");
    }
}

void IxgbeDevice::clean_tx_queue(struct ixgbe_tx_queue *queue) {
    auto clean_index = queue->clean_index;
    auto cur_index = queue->tx_index;

    while (true) {
        // figure out how many descriptors can be cleaned up
        auto cleanable = cur_index - clean_index; // tx_index is always ahead of clean (invariant of our queue)

        if (cleanable < 0) { // handle wrap-around
            cleanable += queue->num_entries;
        }

        if (cleanable < static_cast<int32_t>(TX_CLEAN_BATCH)) {
            break;
        }

        // calculcate the index of the last transcriptor in the clean batch
        // we can't check all descriptors for performance reasons
        int32_t cleanup_to = clean_index + TX_CLEAN_BATCH - 1;
        if (cleanup_to >= queue->num_entries) {
            cleanup_to -= queue->num_entries;
        }

        volatile union ixgbe_adv_tx_desc *txd = queue->descriptors + cleanup_to;
        uint32_t status = txd->wb.status;
        // hardware sets this flag as soon as it's sent out, we can give back all bufs in the batch to the mempool
        if (status & IXGBE_ADVTXD_STAT_DD) {
            if (queue->mempool) {
                auto elements = std::min(queue->bufs_in_use.size(), static_cast<size_t>(TX_CLEAN_BATCH));

                for (size_t i = 0; i < elements; i++) {
                    auto buf = queue->bufs_in_use.front();
                    queue->mempool->free_buf(buf);
                    queue->bufs_in_use.pop_front();
                }
            }
            // next descriptor to be cleaned up is one after the one we just cleaned
            clean_index = wrap_ring(static_cast<uint16_t>(cleanup_to), queue->num_entries);
        } else {
            // clean the whole batch or nothing; yes, this leaves some packets in
            // the queue forever if you stop transmitting, but that's not a real concern
            break;
        }
    }

    queue->clean_index = clean_index;
}

}  // namespace ixy
