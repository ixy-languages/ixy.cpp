#ifndef IXY_IXGBE_HPP
#define IXY_IXGBE_HPP

#include <vector>
#include <unistd.h>

#include "ixy/ixy.hpp"
#include "log.hpp"
#include "pci.hpp"
#include "ixy/mempool.hpp"

namespace ixy {

const unsigned int MAX_QUEUES = 64;

// allocated for each rx queue, keeps state for the receive function
struct ixgbe_rx_queue {
    volatile union ixgbe_adv_rx_desc *descriptors;
    std::shared_ptr<Mempool> mempool;
    uint16_t num_entries;
    // position we are reading from
    uint16_t rx_index;
    // used to map descriptors back to their mbuf for freeing
    std::vector<uint64_t> bufs_in_use;
};

// allocated for each tx queue, keeps state for the transmit function
struct ixgbe_tx_queue {
    volatile union ixgbe_adv_tx_desc *descriptors;
    std::shared_ptr<Mempool> mempool;
    uint16_t num_entries;
    // position to clean up descriptors that where sent out by the nic
    uint16_t clean_index;
    // position to insert packets for transmission
    uint16_t tx_index;
    // used to map descriptors back to their mbuf for freeing
    std::deque<uint64_t> bufs_in_use;
};

class IxgbeDevice : public IxyDevice {
public:
    static auto
    init(const std::string &pci_addr, uint16_t num_rx_queues,
         uint16_t num_tx_queues) -> std::unique_ptr<IxgbeDevice> {
        if (getuid()) {
            warn("not running as root, this will probably fail");
        }

        if (num_rx_queues > MAX_QUEUES) {
            error("cannot configure " << num_rx_queues << " rx queues: limit is " << MAX_QUEUES);
        }

        if (num_tx_queues > MAX_QUEUES) {
            error("cannot configure " << num_tx_queues << " tx queues: limit is " << MAX_QUEUES);
        }

        std::vector<ixgbe_rx_queue> rx_queues;
        std::vector<ixgbe_tx_queue> tx_queues;

        rx_queues.reserve(num_rx_queues);
        tx_queues.reserve(num_tx_queues);

        auto mapped_region = pci_map_resource(pci_addr);

        auto addr = mapped_region.first;
        auto len = mapped_region.second;

        auto dev = IxgbeDevice{pci_addr, addr, len, num_rx_queues, num_tx_queues, rx_queues, tx_queues};

        dev.reset_and_init();

        return std::make_unique<IxgbeDevice>(std::move(dev));
    }

    [[nodiscard]] auto get_driver_name() -> const std::string & override;

    [[nodiscard]] auto get_pci_addr() -> const std::string & override;

    [[nodiscard]] auto get_mac_addr() -> std::array<uint8_t, 6> override;

    void set_mac_addr(const std::array<uint8_t, 6> &mac) override;

    auto rx_batch(uint16_t queue_id, std::deque<Packet> &buffer, uint32_t packets) -> uint32_t override;

    auto tx_batch(uint16_t queue_id, std::deque<Packet> &buffer) -> uint32_t override;

    void read_stats(struct device_stats &device_stats) override;

    void reset_stats() override;

    [[nodiscard]] auto get_link_speed() -> uint32_t override;

private:
    IxgbeDevice(std::string pci_addr, uint8_t *addr, uint64_t len, uint16_t num_rx_queues,
                uint16_t num_tx_queues, std::vector<ixgbe_rx_queue> rx_queues,
                std::vector<ixgbe_tx_queue> tx_queues) : pci_addr{move(pci_addr)},
                                                         addr{addr},
                                                         len{len},
                                                         num_rx_queues{num_rx_queues},
                                                         num_tx_queues{num_tx_queues},
                                                         rx_queues{move(rx_queues)},
                                                         tx_queues{move(tx_queues)} {}

    void init_rx();

    void init_tx();

    void start_rx_queue(uint16_t queue_id);

    void start_tx_queue(uint16_t queue_id);

    void set_promisc(bool enabled);

    void init_link();

    void wait_for_link();

    void reset_and_init();

    inline void set_reg32(uint64_t reg, uint32_t value);

    inline auto get_reg32(uint64_t reg) -> uint32_t;

    inline void set_flags32(uint64_t reg, uint32_t flags);

    inline void clear_flags32(uint64_t reg, uint32_t flags);

    inline void wait_clear_reg32(uint64_t reg, uint32_t mask);

    inline void wait_set_reg32(uint64_t reg, uint32_t mask);

    static void clean_tx_queue(struct ixgbe_tx_queue *queue);

    static const std::string driver_name;
    const std::string pci_addr;
    uint8_t *addr;
    uint64_t len;
    uint16_t num_rx_queues;
    uint16_t num_tx_queues;
    std::vector<ixgbe_rx_queue> rx_queues;
    std::vector<ixgbe_tx_queue> tx_queues;
};

}  // namespace ixy

#endif //IXY_IXGBE_HPP
