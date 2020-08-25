#ifndef IXY_STATS_HPP
#define IXY_STATS_HPP

#include <iomanip>
#include <iostream>
#include <ctime>

#include "ixy.hpp"

namespace ixy {

[[nodiscard]] static auto diff_mpps(uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) -> double {
    return (pkts_new - pkts_old) / 1000000.0 / (nanos / 1000000000.0);
}

[[nodiscard]] static auto
diff_mbit(uint64_t bytes_new, uint64_t bytes_old, uint64_t pkts_new, uint64_t pkts_old,
          uint64_t nanos) -> uint32_t {
    // take stuff on the wire into account, i.e., the preamble, SFD and IFG (20 bytes)
    // otherwise it won't show up as 10000 mbit/s with small packets which is confusing
    return static_cast<uint32_t>(((bytes_new - bytes_old) / 1000000.0 / (nanos / 1000000000.0)) * 8 +
                                 diff_mpps(pkts_new, pkts_old, nanos) * 20 * 8);
}

// returns a timestamp in nanoseconds
// based on rdtsc on reasonably configured systems and is hence fast
[[nodiscard]] static auto monotonic_time() -> uint64_t {
    timespec timespec{};
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    return static_cast<uint64_t>(timespec.tv_sec * 1000 * 1000 * 1000 + timespec.tv_nsec);
}

struct device_stats {
    explicit device_stats(IxyDevice &dev) : dev(dev) {
        dev.reset_stats();
    };

    device_stats(const device_stats &) = default;

    auto operator=(const device_stats &other) -> device_stats & {
        dev = other.dev;
        rx_pkts = other.rx_pkts;
        tx_pkts = other.tx_pkts;
        rx_bytes = other.rx_bytes;
        tx_bytes = other.tx_bytes;

        return *this;
    };

    void print_stats() {
        fmt::print("[{}] RX: {} bytes {} packets\n", dev.get_pci_addr(), rx_bytes, rx_pkts);
        fmt::print("[{}] TX: {} bytes {} packets\n", dev.get_pci_addr(), tx_bytes, tx_pkts);
    };

    void print_stats_diff(struct device_stats &stats_new, uint64_t nanos) const {
        fmt::print("[{}] RX: {} Mbit/s {:.2f} Mpps\n", stats_new.dev.get_pci_addr(),
                   diff_mbit(stats_new.rx_bytes, rx_bytes, stats_new.rx_pkts, rx_pkts, nanos),
                   diff_mpps(stats_new.rx_pkts, rx_pkts, nanos));
        fmt::print("[{}] TX: {} Mbit/s {:.2f} Mpps\n", stats_new.dev.get_pci_addr(),
                   diff_mbit(stats_new.tx_bytes, tx_bytes, stats_new.tx_pkts, tx_pkts, nanos),
                   diff_mpps(stats_new.tx_pkts, tx_pkts, nanos));
    };

    IxyDevice &dev;
    size_t rx_pkts{};
    size_t tx_pkts{};
    size_t rx_bytes{};
    size_t tx_bytes{};
};

}  // namespace ixy

#endif //IXY_STATS_HPP
